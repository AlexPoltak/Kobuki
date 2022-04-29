#include "mainwindow.h"
#include "ui_mainwindow.h"


using namespace cv;
using namespace std;
local local;

autonomous autonomous;



float MainWindow::distPointToLine(float point_x, float point_y, float line_x1, float line_y1, float line_x2, float line_y2)
{
     double A = point_x - line_x1;
     double B = point_y - line_y1;
     double C = line_x2 - line_x1;
     double D = line_y2 - line_y1;

     double dot = A * C + B * D;
     double len_sq = C * C + D * D;
     double param = -1;
     if (len_sq != 0) //in case of 0 length line
         param = dot / len_sq;

     double xx, yy;

     if (param < 0) {
       xx = line_x1;
       yy = line_y1;
     }
     else if (param > 1) {
       xx = line_x2;
       yy = line_y2;
     }
     else {
       xx = line_x1 + param * C;
       yy = line_y1 + param * D;
     }

     double dx = point_x - xx;
     double dy = point_y - yy;
     return sqrt(dx * dx + dy * dy);
}



//funkcia local robot je na priame riadenie robota, ktory je vo vasej blizskoti, viete si z dat ratat polohu atd (zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
void MainWindow::localrobot(TKobukiData &sens)
{

    if(local.prvyStart)
    {
        local.angleOnStart=sens.GyroAngle;
        local.prevValEncLeft=sens.EncoderLeft;
        local.prevValEncRight=sens.EncoderRight;
        local.prevValGyro=sens.GyroAngle;
        local.prvyStart=false;

    }

       //overflow of left wheel
        local.is_overflow=sens.EncoderLeft-local.prevValEncLeft;
        if(local.is_overflow<overflow_up){
           local.tr_dist_of_LW= (tickToMeter*(sens.EncoderLeft-local.prevValEncLeft+65536));
           cout <<  "upL" << endl;
        }

        else if(local.is_overflow>overflow_down){
           local.tr_dist_of_LW= (tickToMeter*(sens.EncoderLeft-local.prevValEncLeft-65536));
           cout <<  "downL" << endl;
        }
        else{
            local.tr_dist_of_LW=(tickToMeter*(sens.EncoderLeft-local.prevValEncLeft));
        }
        local.prevValEncLeft=sens.EncoderLeft;


       //overflow of right wheel
        local.is_overflow=sens.EncoderRight-local.prevValEncRight;
        if(local.is_overflow<overflow_up){
           local.tr_dist_of_RW= (tickToMeter*(sens.EncoderRight-local.prevValEncRight+65536));
           cout <<  "upR" << endl;
        }

        else if(local.is_overflow>overflow_down){
           local.tr_dist_of_RW= (tickToMeter*(sens.EncoderRight-local.prevValEncRight-65536));
           cout <<  "downR" << endl;
        }
        else{
            local.tr_dist_of_RW=(tickToMeter*(sens.EncoderRight-local.prevValEncRight));
        }
        local.prevValEncRight=sens.EncoderRight;




    //    //calculation of traveled distance of both wheels from two last positions

        local.tr_dist=(local.tr_dist_of_RW+local.tr_dist_of_LW)/2;
        local.tr_dist_fr_lastP=local.tr_dist_fr_lastP+local.tr_dist;

        //recalculation of gyroangle to be 0 when code start and then it will be counting from this angle(+X is in front of robot,+Y is on left of robot)
        if(sens.GyroAngle-local.angleOnStart<-18000)
        {
            local.gyroAngle_180_180= sens.GyroAngle-local.angleOnStart+36000;
        }
        else if(sens.GyroAngle-local.angleOnStart>18000)
        {
            local.gyroAngle_180_180= sens.GyroAngle-local.angleOnStart-36000;
        }
        else
        {
            local.gyroAngle_180_180= sens.GyroAngle-local.angleOnStart;

        }

        //actual location of robot
        local.robotX=local.robotX+(local.tr_dist*cos((local.gyroAngle_180_180/100.0)* PI / 180.0));
        local.robotY=local.robotY+(local.tr_dist*sin((local.gyroAngle_180_180/100.0)* PI / 180.0));



//        cout<<local.robotX*100<<"gregerg"<<local.robotY*100<<endl;
        //finding of recalculation of Gyroangle to angle from 0 to 360
        if(local.gyroAngle_180_180<0){local.gyroAngle_0_360=local.gyroAngle_180_180+36000;}
        else{local.gyroAngle_0_360=local.gyroAngle_180_180;}

        if(local.prevValGyro-local.gyroAngle_180_180!=0){local.rotating=true;}
        else{local.rotating=false;}



        if(local.requiredPosX.size()>0&&local.prekazka==false){
            //if robot is in surrounding of current setpoints then set all parameters and set new required position
            if(euclidDist(local.robotX,local.robotY,local.requiredPosX.front(),local.requiredPosY.front())  <  autonomous.deadZoneToRequiredPos)
            {
                cout<<"2222222222222"<<endl;
                local.requiredPosX.pop_front();
                local.requiredPosY.pop_front();
                local.checkRequiredPosX.clear();
                local.checkRequiredPosY.clear();

            }
        }



}

// funkcia local laser je naspracovanie dat z lasera(zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
int MainWindow::locallaser(LaserMeasurement &laserData)
{

//    paintThisLidar(laserData);

    local.fusionPoints.clear();

    local.maxMapY=0;
    local.minMapY=10000000;
    local.maxMapX=0;
    local.minMapX=10000000;

    for(int k=0;k<laserData.numberOfScans;k++)
    {   
        if(laserData.Data[k].scanDistance/1000.0<0.3&&laserData.Data[k].scanDistance/1000.0>0.0&&!local.pointSelected)
        {
            local.stoped=true;

            cout<<"stopol som"<<endl;
        }



        double dist=laserData.Data[k].scanDistance/1000.0;

        local.sensorDist[k]=dist;
        if(dist<0.14)
            continue;

        //get laser global points
        double xOfPoint=(dist*cos((360.0-laserData.Data[k].scanAngle+local.gyroAngle_0_360/100.0)*PI/180.0))+local.robotX;
        double yOfPoint=(dist*sin((360.0-laserData.Data[k].scanAngle+local.gyroAngle_0_360/100.0)*PI/180.0))+local.robotY;

        //insert points to map
        if(local.rotating==false&&dist<2.5){
            local.robotMap[(int)round((xOfPoint/0.1)+60.0)][(int)round((yOfPoint/0.1)+60.0)]=1;
            local.robotStartCellX=(local.robotX*10.0)+60.0;
            local.robotStartCellY=(local.robotX*10.0)+60.0;
        }
        cout<<local.checkRequiredPosX.size()<<"     "<<local.prekazka<<endl;
        if(!local.checkRequiredPosX.empty()&&local.prekazka==false){
            Point2f pt(xOfPoint, yOfPoint);
            Point2f midpoint(local.robotX+   ((local.checkRequiredPosX.front()-local.robotX)/2),local.robotY+    ((local.checkRequiredPosY.front()-local.robotY)/2));
            RotatedRect rr1 ( midpoint,Size2f(euclidDist(local.robotX,local.robotY,local.checkRequiredPosX.front(),local.checkRequiredPosY.front()),0.4), atan2(local.checkRequiredPosY.front() - local.robotY, local.checkRequiredPosX.front() - local.robotX)*180/PI);
            Point2f vtx[4];

            rr1.points(vtx);
            if(pointInRectangle(vtx[1],vtx[0],vtx[3],vtx[2],pt)&&abs(pt.x)>0&&abs(pt.y)>0){
                local.prekazka=true;
                cout<<"prekazka"<<endl;
            }

        }

    }


    if(local.pointSelected&&!local.prekazka){
        autonomous.requiredPosX.push_back(local.checkRequiredPosX.front());
        autonomous.requiredPosY.push_back(local.checkRequiredPosY.front());
        local.endOfPositioning=false;
        local.pointSelected=false;

    }

    if(local.stoped==true&&!local.checkRequiredPosX.empty()&&local.dobre){
        local.checkRequiredPosX.clear();
        local.checkRequiredPosY.clear();
        local.selectedPoints.clear();
        local.endOfPositioning=false;
        ui->Warning_Prekazka_text->setText("skoro som narazil");
        ui->Warning_Prekazka_text->setVisible(true);
        local.dobre=false;
        local.mozes=true;
    }

    if(!local.prekazka&&!local.checkRequiredPosX.empty()&&!local.pointSelected){
        cout<<"ides"<<endl;
        autonomous.requiredPosX.push_back(local.checkRequiredPosX.front());
        autonomous.requiredPosY.push_back(local.checkRequiredPosY.front());
        local.requiredPosX.push_back(local.checkRequiredPosX.front());
        local.requiredPosY.push_back(local.checkRequiredPosY.front());

        local.endOfPositioning=false;
    }


//    if(stoj==true&&!checkRequiredPosX.empty()&&euclidDist(robotX,robotY,checkRequiredPosX.front(),checkRequiredPosY.front())>0.3){dobre=true;stoj=false;}

    if(local.prekazka==true)
    {
        ui->Warning_Prekazka_text->setText("V ceste je prekážka, naviguj na iné miesto");
        ui->Warning_Prekazka_text->setVisible(true);
        local.endOfPositioning=true;
        local.selectedPoints.clear();
        local.prekazka=false;

    }

    if(local.requiredPosY.empty()&&local.selectedPoints.size()>0){
        local.selectedPoints.clear();
    }


    for(int k=0;k<120;k++){
        for(int l=0;l<120;l++){
            if(local.robotMap[k][l]==1){
                if(local.maxMapX<k){local.maxMapX=k;}
                if(local.minMapX>k){local.minMapX=k;}
                if(local.maxMapY<l){local.maxMapY=l;}
                if(local.minMapY>l){local.minMapY=l;}
            }
        }
    }
    if(dl%2==0)
    {

    }
    dl++;



//    fusion points calculation
    local.maxdist=0;
    for(int k=0;k<paintLaserData.numberOfScans;k++){
        double dist=laserData.Data[k].scanDistance/1000.0;
        if(dist*10>local.maxdist){local.maxdist=dist*10;}

        if(paintLaserData.Data[k].scanAngle>=333||paintLaserData.Data[k].scanAngle<=27){

            double fuziaZ=paintLaserData.Data[k].scanDistance/10.0*cos((paintLaserData.Data[k].scanAngle)*PI/180.0);
            double fuziaX=paintLaserData.Data[k].scanDistance/10.0*sin((paintLaserData.Data[k].scanAngle)*PI/180.0);

            local.xObr=(robotPicture.cols/2.0)-((628.036*fuziaX)/fuziaZ);
            local.yObr=(robotPicture.rows/2.0)+((628.036*local.fuziaY)/fuziaZ);


            double dist=paintLaserData.Data[k].scanDistance/1000.0-0.3;
            if(dist<=0.1)
            {
                local.fusionPoints.push_back(make_tuple(local.xObr,local.yObr,"red",dist*10));
            }
            else if(dist>0.1&&dist<=0.2)
            {
                local.fusionPoints.push_back(make_tuple(local.xObr,local.yObr,"orange",dist*10));
            }
            else if(dist>0.2&&dist<=0.5)
            {
                local.fusionPoints.push_back(make_tuple(local.xObr,local.yObr,"yellow",dist*10));
            }
            else
            {
                local.fusionPoints.push_back(make_tuple(local.xObr,local.yObr,"gray",dist*10));
            }

        }
    }

    paintThisLidar(laserData);

    return -1;
}


double MainWindow::ramp(double speed,double inc,bool *start)
{
    if(*start==true){
        double s=autonomous.incPer;

        if(speed>=0){
            autonomous.incPer=autonomous.incPer+inc;
        }
        else{autonomous.incPer=autonomous.incPer-inc;}

        if(abs(s)>abs(speed))
        {
           autonomous.incPer=0;
           *start=false;
           return speed;
        }
        return s;
    }
    else{
        return false;
    }

}

double MainWindow::euclidDist(double x1,double y1,double x2,double y2)
{
   return sqrt(pow((x2-x1),2)+pow((y2-y1),2));
}

Point2f vect2d(cv::Point2f p1, Point2f p2) {
    Point2f temp;
    temp.x=(p2.x - p1.x);
    temp.y=(-1 * (p2.y - p1.y));
    return temp;}

bool MainWindow::pointInRectangle(Point2f A, Point2f B, Point2f C, Point2f D, Point2f m ) {
    Point2f AB = vect2d(A, B);  float C1 = -1 * (AB.y*A.x + AB.x*A.y); float  D1 = (AB.y*m.x + AB.x*m.y) + C1;
    Point2f AD = vect2d(A, D);  float C2 = -1 * (AD.y*A.x + AD.x*A.y); float D2 = (AD.y*m.x + AD.x*m.y) + C2;
    Point2f BC = vect2d(B, C);  float C3 = -1 * (BC.y*B.x + BC.x*B.y); float D3 = (BC.y*m.x + BC.x*m.y) + C3;
    Point2f CD = vect2d(C, D);  float C4 = -1 * (CD.y*C.x + CD.x*C.y); float D4 = (CD.y*m.x + CD.x*m.y) + C4;
    return     0 >= D1 && 0 >= D4 && 0 <= D2 && 0 >= D3;}

bool MainWindow::inRange(double low, double high, double x)
{
    if(low <= x && x <= high){return true;}
    else {return false;}
}

void MainWindow::go(double speed)
{
    sendRobotCommand(ROBOT_VPRED,speed);

}
void MainWindow::turn_around(double angular_velocity)
{
    std::vector<unsigned char> mess=robot.setRotationSpeed(angular_velocity);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}


void MainWindow::stop()
{
    sendRobotCommand(ROBOT_STOP);
}


void MainWindow::paintThisLidar(LaserMeasurement &laserData)
{
    memcpy( &paintLaserData,&laserData,sizeof(LaserMeasurement));
    updateLaserPicture=1;
    update();
}




























//--autonomousrobot simuluje slucku robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad polohovy regulator, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
void MainWindow::autonomousrobot(TKobukiData &sens)
{

    if(prvyStart)
    {
        autonomous.angleOnStart=sens.GyroAngle;
        autonomous.prevValEncLeft=sens.EncoderLeft;
        autonomous.prevValEncRight=sens.EncoderRight;
//        autonomous.prevValGyro=sens.GyroAngle;

        prvyStart=false;

    }
    if(!autonomous.centralSTOP){
       //overflow of left wheel
        autonomous.is_overflow=sens.EncoderLeft-autonomous.prevValEncLeft;
        if(autonomous.is_overflow<overflow_up){
           autonomous.tr_dist_of_LW= (tickToMeter*(sens.EncoderLeft-autonomous.prevValEncLeft+65536));
           cout <<  "upL" << endl;
        }

        else if(autonomous.is_overflow>overflow_down){
           autonomous.tr_dist_of_LW= (tickToMeter*(sens.EncoderLeft-autonomous.prevValEncLeft-65536));
           cout <<  "downL" << endl;
        }
        else{
            autonomous.tr_dist_of_LW=(tickToMeter*(sens.EncoderLeft-autonomous.prevValEncLeft));
        }
        autonomous.prevValEncLeft=sens.EncoderLeft;


       //overflow of right wheel
        autonomous.is_overflow=sens.EncoderRight-autonomous.prevValEncRight;
        if(autonomous.is_overflow<overflow_up){
           autonomous.tr_dist_of_RW= (tickToMeter*(sens.EncoderRight-autonomous.prevValEncRight+65536));
           cout <<  "upR" << endl;
        }

        else if(autonomous.is_overflow>overflow_down){
           autonomous.tr_dist_of_RW= (tickToMeter*(sens.EncoderRight-autonomous.prevValEncRight-65536));
           cout <<  "downR" << endl;
        }
        else{
            autonomous.tr_dist_of_RW=(tickToMeter*(sens.EncoderRight-autonomous.prevValEncRight));
        }
        autonomous.prevValEncRight=sens.EncoderRight;




    //    //calculation of traveled distance of both wheels from two last positions

        autonomous.tr_dist=(autonomous.tr_dist_of_RW+autonomous.tr_dist_of_LW)/2;
        autonomous.tr_dist_fr_lastP=autonomous.tr_dist_fr_lastP+autonomous.tr_dist;

        //recalculation of gyroangle to be 0 when code start and then it will be counting from this angle(+X is in front of robot,+Y is on left of robot)
        if(sens.GyroAngle-autonomous.angleOnStart<-18000)
        {
            autonomous.gyroAngle_180_180= sens.GyroAngle-autonomous.angleOnStart+36000;
        }
        else if(sens.GyroAngle-autonomous.angleOnStart>18000)
        {
            autonomous.gyroAngle_180_180= sens.GyroAngle-autonomous.angleOnStart-36000;
        }
        else
        {
            autonomous.gyroAngle_180_180= sens.GyroAngle-autonomous.angleOnStart;

        }

        //actual location of robot
        autonomous.robotX=autonomous.robotX+(autonomous.tr_dist*cos((autonomous.gyroAngle_180_180/100.0)* PI / 180.0));
        autonomous.robotY=autonomous.robotY+(autonomous.tr_dist*sin((autonomous.gyroAngle_180_180/100.0)* PI / 180.0));

        //finding of recalculation of Gyroangle to angle from 0 to 360
        if(autonomous.gyroAngle_180_180<0){autonomous.gyroAngle_0_360=autonomous.gyroAngle_180_180+36000;}
        else{autonomous.gyroAngle_0_360=autonomous.gyroAngle_180_180;}






    if(autonomous.requiredPosX.size()>0&&autonomous.prekazka==false&&!autonomous.stoped){
        //if robot is in surrounding of current setpoints then set all parameters and set new required position
        if(euclidDist(autonomous.robotX,autonomous.robotY,autonomous.requiredPosX.front(),autonomous.requiredPosY.front())  <  autonomous.deadZoneToRequiredPos)
        {
            autonomous.startX=autonomous.robotX;
            autonomous.startY=autonomous.robotY;
            autonomous.startOfTranslate=true;
            autonomous.startOfRotate=true;
            autonomous.tr_dist_fr_lastP=0;
            autonomous.requiredPosX.pop_front();
            autonomous.requiredPosY.pop_front();

            local.checkRequiredPosX.clear();
            local.checkRequiredPosY.clear();

            autonomous.dobre=true;
            autonomous.stoj=false;


            if((autonomous.requiredPosX.empty()||autonomous.requiredPosY.empty())&&autonomous.endOfPositioning==false)
            {
                autonomous.endOfPositioning=true;
                cout<<"end Of Positioning"<<endl;
                stop();
                printt();

            }
        }



        if(autonomous.endOfPositioning==false){
         //Navigation
            //if distance of robot from trajectory is more than , or if angle of robot from setpoint angle is more than, then recalculate setpoints
            if(abs(distPointToLine(autonomous.robotX,autonomous.robotY,autonomous.startX,autonomous.startY,autonomous.requiredPosX.front(),autonomous.requiredPosY.front()))>autonomous.deadZoneTranslate){autonomous.tr_dist_fr_lastP=0;}
            if(abs(autonomous.setpointAngle-autonomous.gyroAngle_180_180/100.0)>autonomous.deadZone2Angle){autonomous.tr_dist_fr_lastP=0;}


            //finding setpoints for angle and distance to required position
            if(autonomous.tr_dist_fr_lastP==0)
            {
                autonomous.setpointAngle=(atan2(autonomous.requiredPosY.front() -autonomous.robotY,autonomous.requiredPosX.front()-autonomous.robotX)*180.0/PI);

               autonomous. setpointLength=sqrt(pow(autonomous.requiredPosY.front()-autonomous.robotY,2)+pow(autonomous.requiredPosX.front()-autonomous.robotX,2));
            }

            if(autonomous.setpointAngle<0)
            {
                autonomous.setpointAngle_0_360= 360+autonomous.setpointAngle;
            }
            else{autonomous.setpointAngle_0_360=autonomous.setpointAngle;}

            //rotation speed control
            if(abs(autonomous.setpointAngle_0_360-autonomous.gyroAngle_0_360/100.0)>autonomous.deadZone1Angle)
            {
//                cout<<autonomous.setpointAngle_0_360<<" angle"<<autonomous.gyroAngle_0_360<<endl;

                autonomous.rotating=true;
                autonomous.translating=false;

                //this is recalculation to make the robot rotate a shorter path
                 if(autonomous.gyroAngle_180_180/100-autonomous.setpointAngle>-180.0&&autonomous.gyroAngle_180_180/100-autonomous.setpointAngle<0.0)
                 {
                     autonomous.outputAngleAction = autonomous.P_reg_Angle.calculate(autonomous.setpointAngle, sens.GyroAngle/100.0);
                 }
                 else if(autonomous.setpointAngle_0_360-autonomous.gyroAngle_0_360/100.0<=180)
                 {
                     autonomous.outputAngleAction = autonomous.P_reg_Angle.calculate(autonomous.setpointAngle_0_360, autonomous.gyroAngle_0_360/100.0);
                 }
                 else if((autonomous.setpointAngle_0_360-autonomous.gyroAngle_0_360/100.0)>180)
                 {
                     autonomous.outputAngleAction = autonomous.P_reg_Angle.calculate(autonomous.setpointAngle, sens.GyroAngle/100.0);
                 }


    //            when is start of rotate the output from P angle regulator is limited by ramp
                if(autonomous.startOfRotate==true)
                {
                    autonomous.outputAngleAction=ramp(autonomous.outputAngleAction,0.1,&autonomous.startOfRotate);
                    turn_around(autonomous.outputAngleAction);
//                    cout<<autonomous.outputAngleAction<<"rotationspeed1"<<endl;
                }
    //            else turning and deceleration is fully on P angle regulator
                else
                {
                    turn_around(autonomous.outputAngleAction);
//                    cout<<autonomous.outputAngleAction<<"rotationspeed2"<<endl;
                }
            }


            //translation speed control
            if(abs(autonomous.setpointAngle_0_360-autonomous.gyroAngle_0_360/100.0)<=autonomous.deadZone1Angle){


        //        cout<<rotating<<endl;
                //
                autonomous.outputLenAction = autonomous.P_reg_Length.calculate(autonomous.setpointLength, autonomous.tr_dist_fr_lastP)*100.0;

                //when is start of translation the output from P translate regulator is limited by ramp
                if(autonomous.startOfTranslate==true)
                {
                    autonomous.outputLenAction=ramp(autonomous.outputLenAction,10,&autonomous.startOfTranslate);
                    go(autonomous.outputLenAction);
    //                cout<<outputLenAction<<"speed1"<<endl;
                }
                //else translation and deceleration is fully on P translate regulator
                else
                {
                    go(autonomous.outputLenAction);
    //                cout<<outputLenAction<<"speed2"<<endl;
                }
                autonomous.rotating=false;
                autonomous.translating=true;

            }
        }

    }
//    prevValGyro=sens.GyroAngle;

    }










}
//--autonomouslaser simuluje spracovanie dat z robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad sposob obchadzania prekazky, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
int MainWindow::autonomouslaser(LaserMeasurement &laserData)
{

    for(int k=0;k<laserData.numberOfScans;k++)
    {
        if(laserData.Data[k].scanDistance/1000.0<0.3&&laserData.Data[k].scanDistance/1000.0>0.0&&autonomous.mozes)
        {
            sendRobotCommand(ROBOT_STOP);
            autonomous.requiredPosX.clear();
            autonomous.requiredPosY.clear();
            autonomous.stoped=true;
            autonomous.mozes=false;
        }
    }


    for(int k=0;k<laserData.numberOfScans;k++)
    {

        double dist=laserData.Data[k].scanDistance/1000.0;

        if(dist<0.14)
            continue;

        //get laser global points
        double xOfPoint=(dist*cos((360.0-laserData.Data[k].scanAngle+autonomous.gyroAngle_0_360/100.0)*PI/180.0))+autonomous.robotX;
        double yOfPoint=(dist*sin((360.0-laserData.Data[k].scanAngle+autonomous.gyroAngle_0_360/100.0)*PI/180.0))+autonomous.robotY;


        if(!autonomous.requiredPosX.empty()&&autonomous.prekazka==false){
            Point2f pt(xOfPoint, yOfPoint);
            Point2f midpoint(autonomous.robotX+   ((autonomous.requiredPosX.front()-autonomous.robotX)/2),autonomous.robotY+    ((autonomous.requiredPosY.front()-autonomous.robotY)/2));
            RotatedRect rr1 ( midpoint,Size2f(euclidDist(autonomous.robotX,autonomous.robotY,autonomous.requiredPosX.front(),autonomous.requiredPosY.front()),0.4), atan2(autonomous.requiredPosY.front() - autonomous.robotY, autonomous.requiredPosX.front() - autonomous.robotX)*180/PI);
            Point2f vtx[4];

            rr1.points(vtx);
            if(pointInRectangle(vtx[1],vtx[0],vtx[3],vtx[2],pt)&&abs(pt.x)>0&&abs(pt.y)>0){
                autonomous.prekazka=true;
                cout<<"prekazka"<<endl;
            }

        }

    }
    if(!autonomous.mozes&&autonomous.requiredPosX.size()>0&&!autonomous.prekazka){
        autonomous.stoped=false;
    }

    if(!autonomous.prekazka&&!autonomous.requiredPosX.empty()){
//        requiredPosX.push_back(checkRequiredPosX.front());
//        requiredPosY.push_back(checkRequiredPosY.front());
        autonomous.endOfPositioning=false;
//        pointSelected=false;
//        mozes=false;
    }

//    if(stoj==true&&!checkRequiredPosX.empty()&&euclidDist(robotX,robotY,checkRequiredPosX.front(),checkRequiredPosY.front())>0.3){dobre=true;stoj=false;}

    if(autonomous.prekazka==true)
    {
        sendRobotCommand(ROBOT_STOP);
        ui->Warning_Prekazka_text->setText("V ceste je prekážka, naviguj na iné miesto");
        ui->Warning_Prekazka_text->setVisible(true);
        autonomous.requiredPosX.clear();
        autonomous.requiredPosY.clear();
        autonomous.prekazka=false;

    }

    return -1;
}

///kamera nema svoju vlastnu funkciu ktora sa vola, ak chcete niekde pouzit obrazok, aktualny je v premennej
/// robotPicture alebo ekvivalent AutonomousrobotPicture
/// pozor na synchronizaciu, odporucam akonahle chcete robit nieco s obrazkom urobit si jeho lokalnu kopiu
/// cv::Mat frameBuf; robotPicture.copyTo(frameBuf);








void MainWindow::on_showMap_clicked(bool checked)
{
    showCamera=false;
    local.show_Map_or_Camera="map";
    ui->showCam->setStyleSheet("color: white; background-color: rgba(0, 0, 0, 0.7); border-style: outset; border-width: 1px; border-color: beige");
    ui->showMap->setStyleSheet("color: black; background-color: white; border-style: outset; border-width: 1px; border-color: beige");

}

void MainWindow::on_showCam_clicked(bool checked)
{
    showCamera=true;
    local.show_Map_or_Camera="camera";
    ui->showMap->setStyleSheet("color: white; background-color: rgba(0, 0, 0, 0.7); border-style: outset; border-width: 1px; border-color: beige");
    ui->showCam->setStyleSheet("color: black; background-color: white; border-style: outset; border-width: 1px; border-color: beige");
}


//sposob kreslenia na obrazovku, tento event sa spusti vzdy ked sa bud zavola funkcia update() alebo operacny system si vyziada prekreslenie okna





void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect rect(20,120,700,500);
    rect=ui->frame1->geometry();
    painter.drawRect(rect);
    QRect mapp(20,120,700,500);

    QRect camWindowRect(20,120,700,500);
    camWindowRect=ui->camWindow->geometry();


    if(updateCameraPicture==1)
    {

        for(int k=0;k<local.fusionPoints.size();k++)
            {
//            cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(fusionPoints[k])-3,get<1>(fusionPoints[k])+10), cv::Point(robotPicture.cols-get<0>(fusionPoints[k])+3,get<1>(fusionPoints[k])-10), Scalar(0,0,0), -1,LINE_4);

                if(get<2>(local.fusionPoints[k])=="red"){
                    cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(local.fusionPoints[k])-2,get<1>(local.fusionPoints[k])+abs(get<3>(local.fusionPoints[k])-local.maxdist)/2), cv::Point(robotPicture.cols-get<0>(local.fusionPoints[k])+2,get<1>(local.fusionPoints[k])-abs(get<3>(local.fusionPoints[k])-local.maxdist)/2), Scalar(0,0,255), -1,LINE_4);
                }
                if(get<2>(local.fusionPoints[k])=="orange"){
                    cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(local.fusionPoints[k])-2,get<1>(local.fusionPoints[k])+abs(get<3>(local.fusionPoints[k])-local.maxdist)/2), cv::Point(robotPicture.cols-get<0>(local.fusionPoints[k])+2,get<1>(local.fusionPoints[k])-abs(get<3>(local.fusionPoints[k])-local.maxdist)/2), Scalar(0,128,255), -1,LINE_4);
                }
                if(get<2>(local.fusionPoints[k])=="yellow"){
                    cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(local.fusionPoints[k])-2,get<1>(local.fusionPoints[k])+abs(get<3>(local.fusionPoints[k])-local.maxdist)/2), cv::Point(robotPicture.cols-get<0>(local.fusionPoints[k])+2,get<1>(local.fusionPoints[k])-abs(get<3>(local.fusionPoints[k])-local.maxdist)/2), Scalar(0,255,255), -1,LINE_4);
                }
                if(get<2>(local.fusionPoints[k])=="gray"){
                    cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(local.fusionPoints[k])-2,get<1>(local.fusionPoints[k])+abs(get<3>(local.fusionPoints[k])-local.maxdist)/2), cv::Point(robotPicture.cols-get<0>(local.fusionPoints[k])+2,get<1>(local.fusionPoints[k])-abs(get<3>(local.fusionPoints[k])-local.maxdist)/2), Scalar(128,128,128), -1,LINE_4);
                }
            }
        imgIn= QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);
        imgIn.operator=(imgIn.scaled(rect.width(),rect.height(),Qt::KeepAspectRatio,Qt::TransformationMode()));

        local.camWindow=QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);
        local.camWindow.operator=(local.camWindow.scaled(camWindowRect.width(),camWindowRect.height(),Qt::KeepAspectRatio,Qt::TransformationMode()));

        updateCameraPicture=0;

    }

    if(updateLaserPicture==1 )
    {

        local.mapImage=QImage(local.maxMapX-local.minMapX+1,local.maxMapY-local.minMapY+1, QImage::Format_ARGB32);
        local.mapImage.fill(QColor(255,255,255,0));

        for(int k=local.minMapX;k<=local.maxMapX;k++)
        {
            for(int l=local.minMapY;l<=local.maxMapY;l++)
            {
                if(local.robotMap[k][l]==1){
                    local.mapImage.setPixelColor((int)(k-local.minMapX),(int)(l-local.minMapY),QColor(255,131,0,255));
                }
            }
        }
        local.pocetBuniekX=local.maxMapX-local.minMapX;
        local.pocetBuniekY=local.maxMapY-local.minMapY;
        local.Ypixels=0;
        local.pomer=0;
        local.Xpixels=0;
        local.pomerY=0;


        pero.setStyle(Qt::SolidLine);
        pero.setWidth(3);
        pero.setColor(Qt::white);
        painter.setPen(pero);
        painter.setBrush(QColor(0,0,0,127));

        local.mapImage=local.mapImage.mirrored(0,1);

    }




    if(local.show_Map_or_Camera=="camera"){

        painter.drawImage(rect.x()+(rect.width()-imgIn.width())/2,rect.y(),imgIn);
        QRect mapBackground(10,10,ui->frame1->width()/2.5+20.0,ui->frame1->height()/2.5+20.0);
        painter.drawRect(mapBackground);
        painter.drawImage(20,20,local.mapImage.scaled(ui->frame1->width()/2.5,ui->frame1->height()/2.5,Qt::KeepAspectRatio,Qt::TransformationMode()));
        local.Ypixels=ui->frame1->height()/2.5;
        local.Xpixels=ui->frame1->width()/2.5;

        if((local.Xpixels/local.pocetBuniekX)<(local.Ypixels/local.pocetBuniekY)){
            local.pomer=(double)(local.Xpixels/local.pocetBuniekX);
            local.pomerY=(double)ui->frame1->height()/2.5-(local.pomer*local.pocetBuniekY);

        }
        else{
            local.pomer=(double)(local.Ypixels/local.pocetBuniekY);
        }

        double posunY=((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)-10.0;
        painter.setBrush((Qt::white));
        painter.drawEllipse(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+20-1.5*local.pomer,  ui->frame1->height()/2.5-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)+20-1.5*local.pomer-local.pomerY,3.0*local.pomer,3.0*local.pomer);

        if(local.selectedPoints.size()>0){
            pero.setStyle(Qt::SolidLine);
            pero.setWidth(local.pomer/2);
            pero.setColor(Qt::white);
            painter.setPen(pero);

            painter.drawLine(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+20, ui->frame1->height()/2.5-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)+20-local.pomerY,((get<0>(local.selectedPoints.front())-local.minMapX)*local.pomer)+20 ,ui->frame1->height()/2.5-((get<1>(local.selectedPoints.front())-local.minMapY)*local.pomer)+20-local.pomerY);

            pero.setStyle(Qt::SolidLine);
            pero.setWidth(local.pomer/4);
            painter.setPen(pero);
            painter.setBrush((Qt::red));
            painter.drawEllipse(((get<0>(local.selectedPoints.front())-local.minMapX)*local.pomer)+20-0.75*local.pomer ,ui->frame1->height()/2.5-((get<1>(local.selectedPoints.front())-local.minMapY)*local.pomer)+20-0.75*local.pomer-local.pomerY,1.5*local.pomer,1.5*local.pomer);

        }
        pero.setStyle(Qt::SolidLine);
        pero.setWidth(local.pomer/4);
        pero.setColor(Qt::black);
        painter.setPen(pero);
        int yyy=sin(local.gyroAngle_180_180/100*PI/180)*1.5*local.pomer;
        int xxx=cos(local.gyroAngle_180_180/100*PI/180)*1.5*local.pomer;

        painter.drawLine(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+20, ui->frame1->height()/2.5-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)+20-local.pomerY, ((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+20+xxx,ui->frame1->height()/2.5-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)+20-local.pomerY-yyy);




    }

    if(local.show_Map_or_Camera=="map")
    {
//        fusionPoints.clear();
        painter.drawImage(80,80,local.mapImage.scaled(ui->frame1->width()-360,ui->frame1->height()-160,Qt::KeepAspectRatio,Qt::TransformationMode()));
        local.Ypixels=ui->frame1->height()-160;
        local.Xpixels=ui->frame1->width()-360;

        if((local.Xpixels/local.pocetBuniekX)<(local.Ypixels/local.pocetBuniekY)){
            local.pomer=(double)(local.Xpixels/local.pocetBuniekX);
            local.pomerY=(double)ui->frame1->height()-160-(local.pomer*local.pocetBuniekY);

        }
        else{
            local.pomer=(double)(local.Ypixels/local.pocetBuniekY);
        }

        double posunY=((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer);
        double posunX=(((local.robotX*10.0))*local.pomer)+((60.0-local.minMapX)*local.pomer);
        painter.setBrush((Qt::white));
        painter.drawEllipse(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+80-1.5*local.pomer,  ui->frame1->height()-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)-80-1.5*local.pomer-local.pomerY,3.0*local.pomer,3.0*local.pomer);



        if(local.selectedPoints.size()>0){
            pero.setStyle(Qt::SolidLine);
            pero.setWidth(local.pomer/2);
            pero.setColor(Qt::white);
            painter.setPen(pero);

            painter.drawLine(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+80, ui->frame1->height()-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)-80-local.pomerY,((get<0>(local.selectedPoints.front())-local.minMapX)*local.pomer)+80 ,ui->frame1->height()-((get<1>(local.selectedPoints.front())-local.minMapY)*local.pomer)-80-local.pomerY);

            pero.setStyle(Qt::SolidLine);
            pero.setWidth(local.pomer/4);
            painter.setPen(pero);
            painter.setBrush((Qt::red));
            painter.drawEllipse(((get<0>(local.selectedPoints.front())-local.minMapX)*local.pomer)+80-0.75*local.pomer ,ui->frame1->height()-((get<1>(local.selectedPoints.front())-local.minMapY)*local.pomer)-80-0.75*local.pomer-local.pomerY,1.5*local.pomer,1.5*local.pomer);

        }
        pero.setStyle(Qt::SolidLine);
        pero.setWidth(local.pomer/4);
        pero.setColor(Qt::black);
        painter.setPen(pero);
        int yyy=sin(local.gyroAngle_180_180/100*PI/180)*1.5*local.pomer;
        int xxx=cos(local.gyroAngle_180_180/100*PI/180)*1.5*local.pomer;
        painter.drawLine(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+80, ui->frame1->height()-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)-80-local.pomerY, ((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+80+xxx,ui->frame1->height()-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)-80-local.pomerY-yyy);

        painter.drawImage(camWindowRect.x()+(camWindowRect.width()-local.camWindow.width())/2,camWindowRect.y(),local.camWindow);

    }




    if( showLidar==true)
    {



    }


    if(showSkeleton==false&&ui->widget->isVisible()){
            ui->widget->setVisible(false);
    }

    if(updateSkeletonPicture==1 && showSkeleton==true)
    {
        if(!ui->widget->isVisible()){
            ui->widget->setVisible(true);
        }

        painter.setPen(Qt::red);
        for(int i=0;i<75;i++)
        {
           double  dist_IND_F_TIP_and_MID_F_TIP=euclidDist(kostricka.joints[8].x,kostricka.joints[8].y,kostricka.joints[12].x,kostricka.joints[12].y);
           double  dist_IND_F_MCP_and_MID_F_MCP=euclidDist(kostricka.joints[5].x,kostricka.joints[5].y,kostricka.joints[9].x,kostricka.joints[9].y);
           double  dist_RING_F_TIP_and_WRIST=euclidDist(kostricka.joints[16].x,kostricka.joints[16].y,kostricka.joints[0].x,kostricka.joints[0].y);
           double  dist_RING_F_MCP_and_WRIST=euclidDist(kostricka.joints[13].x,kostricka.joints[13].y,kostricka.joints[0].x,kostricka.joints[0].y);

           double  dist_PINKY_F_TIP_and_WRIST=euclidDist(kostricka.joints[20].x,kostricka.joints[20].y,kostricka.joints[0].x,kostricka.joints[0].y);
           double  dist_PINKY_F_MCP_and_WRIST=euclidDist(kostricka.joints[17].x,kostricka.joints[17].y,kostricka.joints[0].x,kostricka.joints[0].y);


//           double  Ydiff_IND_F_TIP_and_IND_F_MCP=(kostricka.joints[8].y-kostricka.joints[5].y)/(kostricka.joints[8].x-kostricka.joints[5].x);
           double angle=atan2(kostricka.joints[8].y-kostricka.joints[0].y,kostricka.joints[8].x-kostricka.joints[0].x)*180/PI;
//           cout<<angle<<endl;
//            cout<<dist_IND_F_DIP_and_MID_F_DIP/dist_IND_F_MCP_and_MID_F_MCP_<<"zapestie"<<endl;
           if (  (dist_RING_F_TIP_and_WRIST < dist_RING_F_MCP_and_WRIST )  && (dist_PINKY_F_TIP_and_WRIST<dist_PINKY_F_MCP_and_WRIST)  &&(dist_IND_F_TIP_and_MID_F_TIP/dist_IND_F_MCP_and_MID_F_MCP<1.5)&&angle<-70&&angle>-120)   {
//            cout<<"zapestie"<<endl;
            sendRobotCommand(ROBOT_VPRED,250);
            QString str = "vpred";
            emit uiValuesChanged(str);


           }

           else if (  (dist_RING_F_TIP_and_WRIST < dist_RING_F_MCP_and_WRIST )  && (dist_PINKY_F_TIP_and_WRIST<dist_PINKY_F_MCP_and_WRIST)  &&(dist_IND_F_TIP_and_MID_F_TIP/dist_IND_F_MCP_and_MID_F_MCP>1.7)&&angle<-70&&angle>-120)   {
//            cout<<"zapestie"<<endl;
            sendRobotCommand(ROBOT_VZAD,250);
            QString str = "vzad";
            emit uiValuesChanged(str);


           }
           else if (  (dist_RING_F_TIP_and_WRIST < dist_RING_F_MCP_and_WRIST )  && (dist_PINKY_F_TIP_and_WRIST<dist_PINKY_F_MCP_and_WRIST)  &&(dist_IND_F_TIP_and_MID_F_TIP/dist_IND_F_MCP_and_MID_F_MCP<1.5)&&angle<-120)   {
//            cout<<"zapestie"<<endl;
               sendRobotCommand(ROBOT_VLAVO,3.14159/4);
               emit uiValuesChanged("vlavo");

           }

           else if (  (dist_RING_F_TIP_and_WRIST < dist_RING_F_MCP_and_WRIST )  && (dist_PINKY_F_TIP_and_WRIST<dist_PINKY_F_MCP_and_WRIST)  &&(dist_IND_F_TIP_and_MID_F_TIP/dist_IND_F_MCP_and_MID_F_MCP>1.7)&&angle<-120)   {
//            cout<<"zapestie"<<endl;
               sendRobotCommand(ROBOT_VPRAVO,3.14159/4);
               emit uiValuesChanged("vpravo");


           }

           else{
               sendRobotCommand(ROBOT_STOP);
               emit uiValuesChanged("stop");

           }

        }
    }

    QRect robotShowsenzors(20,120,700,500);
    robotShowsenzors=ui->robotShowSensors->geometry();
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(2);
    pero.setColor(Qt::white);
    painter.setPen(pero);
    painter.setBrush(QColor(0,0,0,127));
    painter.drawRect(robotShowsenzors.translated(10,10));
    painter.setBrush(Qt::white);
    pero.setColor(Qt::black);
    painter.setPen(pero);
    painter.drawEllipse((robotShowsenzors.x()+robotShowsenzors.width())-(robotShowsenzors.width()/2)-25   ,(robotShowsenzors.y()+robotShowsenzors.height())-(robotShowsenzors.height()/2)-22,70,70);
    painter.setBrush(Qt::black);

    painter.drawEllipse((robotShowsenzors.x()+robotShowsenzors.width())-(robotShowsenzors.width()/2)+3   ,(robotShowsenzors.y()+robotShowsenzors.height())-(robotShowsenzors.height()/2)-15,15,15);




    QRect rect1((robotShowsenzors.x()+robotShowsenzors.width())-(robotShowsenzors.width()/2)-32,(robotShowsenzors.y()+robotShowsenzors.height())-(robotShowsenzors.height()/2)-28.5,85,85);
    QRect rect2((robotShowsenzors.x()+robotShowsenzors.width())-(robotShowsenzors.width()/2)-45,(robotShowsenzors.y()+robotShowsenzors.height())-(robotShowsenzors.height()/2)-42,110,110);
    QRect rect3((robotShowsenzors.x()+robotShowsenzors.width())-(robotShowsenzors.width()/2)-57,(robotShowsenzors.y()+robotShowsenzors.height())-(robotShowsenzors.height()/2)-55,135,135);

    painter.setBrush(Qt::red);
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(8);
    pero.setColor(Qt::red);
    painter.setPen(pero);
    int r=277/360;

//H
//        if(sensorDist[i]>0.05&&sensorDist[i]<1){

            if(local.sensorDist[0]<0.6&&local.sensorDist[0]>0.05){
                 painter.drawArc(rect3,67*16,46*16);
            }
            if(local.sensorDist[0]<0.5&&local.sensorDist[0]>0.05){
                painter.drawArc(rect2,67*16,44*16);
            }
            if(local.sensorDist[0]<0.4&&local.sensorDist[0]>0.05){
                painter.drawArc(rect1,69*16,42*16);
            }


//LH
            if(local.sensorDist[60]<0.6&&local.sensorDist[60]>0.05){
                painter.drawArc(rect3,127*16,47*16);
            }
            if(local.sensorDist[60]<0.5&&local.sensorDist[60]>0.05){
                painter.drawArc(rect2,127*16,45*16);
            }
            if(local.sensorDist[60]<0.4&&local.sensorDist[60]>0.05){
                painter.drawArc(rect1,129*16,42*16);
            }

//LD
            if(local.sensorDist[80]<0.6&&local.sensorDist[80]>0.05){
                painter.drawArc(rect3,188*16,46*16);
            }
            if(local.sensorDist[80]<0.5&&local.sensorDist[80]>0.05){
                painter.drawArc(rect2,189*16,44*16);
            }
            if(local.sensorDist[80]<0.4&&local.sensorDist[80]>0.05){
                painter.drawArc(rect1,189*16,42*16);
            }

//D

            if(local.sensorDist[138]<0.6&&local.sensorDist[138]>0.05){
                painter.drawArc(rect3,248*16,46*16);
            }
            if(local.sensorDist[138]<0.5&&local.sensorDist[138]>0.05){
                painter.drawArc(rect2,249*16,44*16);
            }
            if(local.sensorDist[138]<0.4&&local.sensorDist[138]>0.05){
                painter.drawArc(rect1,249*16,42*16);
            }


//PD

            if(local.sensorDist[198]<0.6&&local.sensorDist[198]>0.05){
                painter.drawArc(rect3,307*16,46*16);
            }
            if(local.sensorDist[198]<0.5&&local.sensorDist[198]>0.05){
                painter.drawArc(rect2,308*16,44*16);
            }
            if(local.sensorDist[198]<0.4&&local.sensorDist[198]>0.05){
                painter.drawArc(rect1,309*16,42*16);
            }

//PH

            if(local.sensorDist[218]<0.6&&local.sensorDist[218]>0.05){
                painter.drawArc(rect3,7*16,47*16);
            }
            if(local.sensorDist[218]<0.5&&local.sensorDist[218]>0.05){
                painter.drawArc(rect2,8*16,44*16);
            }
            if(local.sensorDist[218]<0.4&&local.sensorDist[218]>0.05){
                painter.drawArc(rect1,9*16,42*16);
            }

}



void MainWindow::mousePressEvent(QMouseEvent *event)
{
    QPoint lastPoint = event->pos();

    if (event->button() == Qt::LeftButton&&local.show_Map_or_Camera=="map"&&lastPoint.x()<(ui->frame1->width()-220)){

        if( ui->Warning_Prekazka_text->isVisible()&&autonomous.centralSTOP==false)
        {
            ui->Warning_Prekazka_text->setText("V ceste je prekážka, naviguj na iné miesto");
            ui->Warning_Prekazka_text->setVisible(false);
            local.prekazka=false;

                local.checkRequiredPosX.clear();
                local.checkRequiredPosY.clear();
                local.requiredPosX.clear();
                local.requiredPosY.clear();


        }
        else if(local.checkRequiredPosX.empty()&&local.requiredPosX.empty()&&autonomous.centralSTOP==false){

            local.Xnavigate=((lastPoint.x()-80.0)/local.pomer)+local.minMapX;
            local.Ynavigate=local.maxMapY-((lastPoint.y()-80.0)/local.pomer);

            local.checkRequiredPosX.push_back((local.Xnavigate-60.0)/10.0);
            local.checkRequiredPosY.push_back((local.Ynavigate-60.0)/10.0);
            local.selectedPoints.push_back(make_tuple(local.Xnavigate,local.Ynavigate));
            cout<<"goooooo2222o"<<endl;

            if(local.stoped){
                local.pointSelected=true;
                local.stoped=false;
                cout<<"gooooooo"<<endl;
            }
            local.mozes=true;
        }
        else if(autonomous.centralSTOP==true){
            ui->Warning_Prekazka_text->setText("Ak chces navigovat, musis vypnut central STOP");
            ui->Warning_Prekazka_text->setVisible(true);

        }
//        if(local.stoped){
//            local.stoped=false;
//        }



    }
}



void MainWindow::printt()
{

ofstream mapFile;

mapFile.open("example35.txt");

    for (int i=120; i> 0;i--) //This variable is for each row below the x
    {
        for (int j=0; j<120;j++)
        {
            if(local.robotMap[j][i]==1){
                mapFile << "*";
                mapFile << "-";
            }
        }
        mapFile<<std::endl;

    }
    mapFile.close();
}

void MainWindow::on_pushButton_12_clicked()
{
    if(!autonomous.centralSTOP){
        ui->pushButton_12->setStyleSheet("color: #FFF; background-color: #f54242; border: 2px solid #f54242; border-radius: 2px;");
        autonomous.requiredPosX.clear();
        autonomous.requiredPosY.clear();
        local.requiredPosX.clear();
        local.requiredPosY.clear();
        local.checkRequiredPosX.clear();
        local.checkRequiredPosY.clear();
        local.selectedPoints.clear();
        autonomous.endOfPositioning=true;
        stop();
        autonomous.centralSTOP=true;
    }
    else{
        ui->pushButton_12->setStyleSheet("color: #f54242; background-color: #FFF; border: 2px solid #f54242; border-radius: 2px;");
        autonomous.centralSTOP=false;
        ui->Warning_Prekazka_text->setVisible(false);
        autonomous.endOfPositioning=false;


    }
}






///konstruktor aplikacie, nic co tu je nevymazavajte, ak potrebujete, mozete si tu pridat nejake inicializacne parametre
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->Warning_Prekazka_text->setVisible(false);

    connect(this,SIGNAL(uiValuesChanged(QString)),this,SLOT(setUiValues(QString)));


    local.robotX=0;
    local.robotY=0;
    autonomous.robotX=0;
    autonomous.robotY=0;
    robotFi=0;

    showCamera=true;
    showLidar=true;
    showSkeleton=false;
    applyDelay=true;
    dl=0;
    stopall=1;
    prvyStart=true;
    updateCameraPicture=0;
    ipaddress="127.0.0.1";
    std::function<void(void)> f =std::bind(&robotUDPVlakno, (void *)this);
    robotthreadHandle=std::move(std::thread(f));
    std::function<void(void)> f2 =std::bind(&laserUDPVlakno, (void *)this);
    laserthreadHandle=std::move(std::thread(f2));


    std::function<void(void)> f3 =std::bind(&skeletonUDPVlakno, (void *)this);
    skeletonthreadHandle=std::move(std::thread(f3));

    //--ak by ste nahodou chceli konzolu do ktorej mozete vypisovat cez std::cout, odkomentujte nasledujuce dva riadky
   // AllocConsole();
   // freopen("CONOUT$", "w", stdout);


    QFuture<void> future = QtConcurrent::run([=]() {
        imageViewer();
        // Code in this block will run in another thread
    });



        Imager.start();

}

///funkcia co sa zavola ked stlacite klavesu na klavesnici..
/// pozor, ak niektory widget akceptuje klavesu, sem sa nemusite (ale mozete) dostat
/// zalezi na to ako konkretny widget spracuje svoj event
void MainWindow::keyPressEvent(QKeyEvent* event)
{
    //pre pismena je key ekvivalent ich ascii hodnoty
    //pre ine klavesy pozrite tu: https://doc.qt.io/qt-5/qt.html#Key-enum
    std::cout<<event->key()<<std::endl;

    if(event->key()==Qt::Key_W){
        sendRobotCommand(ROBOT_VPRED,250);
    }
    if(event->key()==Qt::Key_A){
        sendRobotCommand(ROBOT_VLAVO,3.14159/4);
    }
    if(event->key()==Qt::Key_S){
        sendRobotCommand(ROBOT_VZAD,250);
    }
    if(event->key()==Qt::Key_D){
        sendRobotCommand(ROBOT_VPRAVO,3.14159/4);
    }
    if(event->key()==Qt::Key_Q){
        sendRobotCommand(ROBOT_STOP);
    }



}

void  MainWindow::setUiValues(QString instruction)
{
     ui->skeletonText->setText(instruction);
}
//--cokolvek za tymto vas teoreticky nemusi zaujimat, su tam len nejake skarede kody































































MainWindow::~MainWindow()
{
    stopall=0;
    laserthreadHandle.join();
    robotthreadHandle.join();
    skeletonthreadHandle.join();
    delete ui;
}









void MainWindow::robotprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    rob_slen = sizeof(las_si_other);
    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    DWORD timeout=100;

    setsockopt(rob_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
#ifdef _WIN32
    Sleep(100);
#else
    usleep(100*1000);
#endif
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    while(stopall==1)
    {

        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {

            continue;
        }
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);

        int returnval=robot.fillData(sens,(unsigned char*)buff);
        if(returnval==0)
        {
            //     memcpy(&sens,buff,sizeof(sens));

            std::chrono::steady_clock::time_point timestampf=std::chrono::steady_clock::now();

            autonomousrobot(sens);

            if(applyDelay==true)
            {
                struct timespec t;
                RobotData newcommand;
                newcommand.sens=sens;
                //    memcpy(&newcommand.sens,&sens,sizeof(TKobukiData));
                //        clock_gettime(CLOCK_REALTIME,&t);
                newcommand.timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                sensorQuerry.push_back(newcommand);
                for(int i=0;i<sensorQuerry.size();i++)
                {
                    if(( std::chrono::duration_cast<std::chrono::nanoseconds>(timestampf-sensorQuerry[i].timestamp)).count()>(2.5*1000000000))
                    {
                        localrobot(sensorQuerry[i].sens);
                        sensorQuerry.erase(sensorQuerry.begin()+i);
                        i--;
                        break;

                    }
                }

            }
            else
            {
                sensorQuerry.clear();
                localrobot(sens);
            }
        }


    }

    std::cout<<"koniec thread2"<<std::endl;
}
/// vravel som ze vas to nemusi zaujimat. tu nic nieje
/// nosy litlle bastard
void MainWindow::laserprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char las_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    setsockopt(las_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#else
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#endif
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, rob_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(stopall==1)
    {

        if ((las_recv_len = recvfrom(las_s, (char *)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, &las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        int returnValue=autonomouslaser(measure);

        if(applyDelay==true)
        {
            struct timespec t;
            LidarVector newcommand;
            memcpy(&newcommand.data,&measure,sizeof(LaserMeasurement));
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            lidarQuerry.push_back(newcommand);
            for(int i=0;i<lidarQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-lidarQuerry[i].timestamp)).count()>(2.5*1000000000))
                {
                    returnValue=locallaser(lidarQuerry[i].data);
                    if(returnValue!=-1)
                    {
                        //sendRobotCommand(returnValue);
                    }
                    lidarQuerry.erase(lidarQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {


            returnValue=locallaser(measure);
            if(returnValue!=-1)
            {
                //sendRobotCommand(returnValue);
            }
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}


void MainWindow::sendRobotCommand(char command,double speed,int radius)
{
    globalcommand=command;
 //   if(applyDelay==false)
    {

        std::vector<unsigned char> mess;
        switch(command)
        {
        case  ROBOT_VPRED:
            mess=robot.setTranslationSpeed(speed);
            break;
        case ROBOT_VZAD:
            mess=robot.setTranslationSpeed(-speed);
            break;
        case ROBOT_VLAVO:
            mess=robot.setRotationSpeed(speed);
            break;
        case ROBOT_VPRAVO:
            mess=robot.setRotationSpeed(-speed);
            break;
        case ROBOT_STOP:
            mess=robot.setTranslationSpeed(0);
            break;
        case ROBOT_ARC:
            mess=robot.setArcSpeed(speed,radius);
            break;


        }
        if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
        {

        }

    }
  /*  else
    {
        struct timespec t;
        RobotCommand newcommand;
        newcommand.command=command;
        newcommand.radius=radius;
        newcommand.speed=speed;
        //clock_gettime(CLOCK_REALTIME,&t);
        newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
        commandQuery.push_back(newcommand);
    }*/
}
/*void MainWindow::autonomousRobotCommand(char command,double speed,int radius)
{
    return;
    std::vector<unsigned char> mess;
    switch(command)
    {
    case  ROBOT_VPRED:
        mess=robot.setTranslationSpeed(speed);
        break;
    case ROBOT_VZAD:
        mess=robot.setTranslationSpeed(speed);
        break;
    case ROBOT_VLAVO:
        mess=robot.setRotationSpeed(speed);
        break;
    case ROBOT_VPRAVO:
        mess=robot.setRotationSpeed(speed);
        break;
    case ROBOT_STOP:
        mess=robot.setTranslationSpeed(0);
        break;
    case ROBOT_ARC:
        mess=robot.setArcSpeed(speed,radius);
        break;

    }
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}
void MainWindow::robotexec()
{


    if(applyDelay==true)
    {
        struct timespec t;

        // clock_gettime(CLOCK_REALTIME,&t);
        auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
        for(int i=0;i<commandQuery.size();i++)
        {
       //     std::cout<<(std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()<<std::endl;
            if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()>(2.5*1000000000))
            {
                char cmd=commandQuery[i].command;
                std::vector<unsigned char> mess;
                switch(cmd)
                {
                case  ROBOT_VPRED:
                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VZAD:
                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VLAVO:
                    mess=robot.setRotationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VPRAVO:
                    mess=robot.setRotationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_STOP:
                    mess=robot.setTranslationSpeed(0);
                    break;
                case ROBOT_ARC:
                    mess=robot.setArcSpeed(commandQuery[i].speed,commandQuery[i].radius);
                    break;

                }
                if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
                {

                }
                commandQuery.erase(commandQuery.begin()+i);
                i--;

            }
        }
    }
}
*/



//void MainWindow::on_pushButton_8_clicked()//forward
//{
//    CommandVector help;
//    help.command.commandType=1;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=0;
//    help.command.desiredDist=100;
//    struct timespec t;

//    // clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_10_clicked()//right
//{
//    CommandVector help;
//    help.command.commandType=2;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=-20;
//    help.command.desiredDist=0;
//    struct timespec t;

//    // clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_11_clicked()//back
//{
//    CommandVector help;
//    help.command.commandType=1;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=0;
//    help.command.desiredDist=-100;
//    struct timespec t;

//    //   clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_9_clicked()//left
//{
//    CommandVector help;
//    help.command.commandType=2;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=20;
//    help.command.desiredDist=0;
//    struct timespec t;

//    //   clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}



void MainWindow::skeletonprocess()
{

    std::cout<<"init skeleton"<<std::endl;
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    ske_slen = sizeof(ske_si_other);
    if ((ske_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char ske_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    std::cout<<setsockopt(ske_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout)<<std::endl;
    std::cout<<setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene))<<std::endl;
#else
    setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene));
#endif
    // zero out the structure
    memset((char *) &ske_si_me, 0, sizeof(ske_si_me));

    ske_si_me.sin_family = AF_INET;
    ske_si_me.sin_port = htons(23432);
    ske_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    ske_si_posli.sin_family = AF_INET;
    ske_si_posli.sin_port = htons(23432);
    ske_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    std::cout<<::bind(ske_s , (struct sockaddr*)&ske_si_me, sizeof(ske_si_me) )<<std::endl;;
    char command=0x00;

    skeleton bbbk;
    double measure[225];
    while(stopall==1)
    {

        if ((ske_recv_len = ::recvfrom(ske_s, (char *)&bbbk.joints, sizeof(char)*1800, 0, (struct sockaddr *) &ske_si_other, &ske_slen)) == -1)
        {

        //    std::cout<<"problem s prijatim"<<std::endl;
            continue;
        }


        memcpy(kostricka.joints,bbbk.joints,1800);
     updateSkeletonPicture=1;
  //      std::cout<<"doslo "<<ske_recv_len<<std::endl;
      //  continue;
        for(int i=0;i<75;i+=3)
        {
        //    std::cout<<klby[i]<<" "<<bbbk.joints[i].x<<" "<<bbbk.joints[i].y<<" "<<bbbk.joints[i].z<<std::endl;
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}

//void MainWindow::on_checkBox_2_clicked(bool checked)
//{
//    showLidar=checked;
//}


void MainWindow::on_checkBox_skeleton_clicked(bool checked)
{
    showSkeleton=checked;
    if(checked){
       ui->checkBox_skeleton->setStyleSheet("color: black; background-color: white; border-style: outset; border-width: 1px; border-color: beige");

    }
    if(!checked){
       ui->checkBox_skeleton->setStyleSheet("color: white; background-color: rgba(0, 0, 0, 0.7); border-style: outset; border-width: 1px; border-color: beige");

    }
}

void MainWindow::on_checkBox_clicked(bool checked)
{
//    applyDelay=checked;
}



void MainWindow::imageViewer()
{
    cv::VideoCapture cap;
    cap.open("http://127.0.0.1:8889/stream.mjpg");
    cv::Mat frameBuf;
    while(1)
    {
        cap >> frameBuf;


        if(frameBuf.rows<=0)
        {
//            std::cout<<"nefunguje"<<std::endl;
            continue;
        }

        if(applyDelay==true)
        {
            struct timespec t;
            CameraVector newcommand;
            frameBuf.copyTo(newcommand.data);
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            cameraQuerry.push_back(newcommand);
            for(int i=0;i<cameraQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-cameraQuerry[i].timestamp)).count()>(2.5*1000000000))
                {

                    cameraQuerry[i].data.copyTo(robotPicture);
                    cameraQuerry.erase(cameraQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {


           frameBuf.copyTo(robotPicture);
        }
        frameBuf.copyTo(AutonomousrobotPicture);
        updateCameraPicture=1;

        update();
//        std::cout<<"vycital som"<<std::endl;
       // cv::imshow("client",frameBuf);
        cv::waitKey(1);
        QCoreApplication::processEvents();
    }
}













