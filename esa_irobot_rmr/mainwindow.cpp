#include "mainwindow.h"
#include <stdio.h>


using namespace cv;
using namespace std;

local local;



//funkcia local robot je na priame riadenie robota, ktory je vo vasej blizskoti, viete si z dat ratat polohu atd (zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
void MainWindow::localrobot(TKobukiData &robotdata)
{

    if(prvyStart)
    {
        local.angleOnStart=robotdata.GyroAngle;
        local.prevValEncLeft=robotdata.EncoderLeft;
        local.prevValEncRight=robotdata.EncoderRight;
        local.prevValGyro=robotdata.GyroAngle;

        prvyStart=false;

    }





//-------------------------------------------------------Localization---------------------------------------------------------





   //recalculate if is overflow of left wheel
    local.is_overflow=robotdata.EncoderLeft-local.prevValEncLeft;
    if(local.is_overflow<overflow_up){
       local.tr_dist_of_LW= (tickToMeter*(robotdata.EncoderLeft-local.prevValEncLeft+65536));
    }
    else if(local.is_overflow>overflow_down){
       local.tr_dist_of_LW= (tickToMeter*(robotdata.EncoderLeft-local.prevValEncLeft-65536));
    }
    else{
       local.tr_dist_of_LW=(tickToMeter*(robotdata.EncoderLeft-local.prevValEncLeft));
    }
    local.prevValEncLeft=robotdata.EncoderLeft;


   //recalculate if is overflow of right wheel
    local.is_overflow=robotdata.EncoderRight-local.prevValEncRight;
    if(local.is_overflow<overflow_up){
       local.tr_dist_of_RW= (tickToMeter*(robotdata.EncoderRight-local.prevValEncRight+65536));
    }

    else if(local.is_overflow>overflow_down){
       local.tr_dist_of_RW= (tickToMeter*(robotdata.EncoderRight-local.prevValEncRight-65536));
    }
    else{
        local.tr_dist_of_RW=(tickToMeter*(robotdata.EncoderRight-local.prevValEncRight));
    }
    local.prevValEncRight=robotdata.EncoderRight;




   //calculation of traveled distance of both wheels
    local.tr_dist=(local.tr_dist_of_RW+local.tr_dist_of_LW)/2;
   //calculation of traveled distance from last point where were recalculate setpoints for P controllers
    local.tr_dist_fr_lastP=local.tr_dist_fr_lastP+local.tr_dist;

   //recalculation of gyroangle to be 0 when code start(+X is in front of robot,+Y is on left of robot), and recalculate if is overflow
    if(robotdata.GyroAngle-local.angleOnStart<-18000)
    {
        local.gyroAngle_180_180= robotdata.GyroAngle-local.angleOnStart+36000;
    }
    else if(robotdata.GyroAngle-local.angleOnStart>18000)
    {
        local.gyroAngle_180_180= robotdata.GyroAngle-local.angleOnStart-36000;
    }
    else
    {
        local.gyroAngle_180_180= robotdata.GyroAngle-local.angleOnStart;
    }


   //recalculation of Gyroangle to angle from 0 to 360
    if(local.gyroAngle_180_180<0){local.gyroAngle_0_360=local.gyroAngle_180_180+36000;}
    else{local.gyroAngle_0_360=local.gyroAngle_180_180;}



   //actual location of robot
    local.robotX=local.robotX+(local.tr_dist*cos((local.gyroAngle_180_180/100.0)* PI / 180.0));
    local.robotY=local.robotY+(local.tr_dist*sin((local.gyroAngle_180_180/100.0)* PI / 180.0));




//-------------------------------------------------------Navigation---------------------------------------------------------





    if(local.requiredPosX.empty()&&local.prevValGyro!=robotdata.GyroAngle){stop();}

    if(!local.requiredPosX.empty()){

       // If robot is in defined surrounding of required position then reset all parameters and set new required position
        if(euclidDist(local.robotX,local.robotY,local.requiredPosX.front(),local.requiredPosY.front())  <  local.deadZoneToRequiredPos)
        {
            local.startX=local.robotX;
            local.startY=local.robotY;
            local.startOfTranslate=true;
            local.startOfRotate=true;
            local.tr_dist_fr_lastP=0;
            local.requiredPosX.pop_front();
            local.requiredPosY.pop_front();



            if(!local.selectedPoints.empty()){
                local.selectedPoints.pop_back();
            }
            if(MODE==2&&local.prekazka&&!local.requiredPosY.empty()){
                local.prekazka=false;
                local.mozes=true;
                local.maxMapY=0;
                local.minMapY=10000000;
                local.maxMapX=0;
                local.minMapX=10000000;
                if(!local.requiredPosX.empty()){
                    local.checkRequiredPosX.clear();
                    local.checkRequiredPosY.clear();
                    local.checkRequiredPosX.push_front(local.requiredPosX.front());
                    local.checkRequiredPosY.push_front(local.requiredPosY.front());
                }
            }

           // If there are no more positions to navigate, then the positioning is over
            if((local.requiredPosX.empty()||local.requiredPosY.empty())&&local.endOfPositioning==false)
            {
                local.endOfPositioning=true;
                local.checkRequiredPosX.clear();
                local.checkRequiredPosY.clear();
                local.selectedPoints.clear();
                cout<<"end Of Positioning"<<endl;
                stop();
                printMapToFile();

            }
        }

        if(!local.requiredPosX.empty()){
           //When distance of robot from trajectory is more than deadzone then recalculate setpoints for P controllers.
            if(abs(distFromPointToLine(local.robotX,local.robotY,local.startX,local.startY,local.requiredPosX.front(),local.requiredPosY.front()))>local.deadZoneTranslate){local.tr_dist_fr_lastP=0;}
           //When the difference between the angle of the robot and the setpoint is more than deadzone then recalculate setpoints for P controllers.
            if(abs(local.setpointAngle-local.gyroAngle_180_180/100.0)>local.deadZone2Angle){local.tr_dist_fr_lastP=0;}


           //finding setpoints for angle and distance to required position
            if(local.tr_dist_fr_lastP==0)
            {
                local.setpointAngle=(atan2(local.requiredPosY.front() -local.robotY,local.requiredPosX.front()-local.robotX)*180.0/PI);
                local.setpointLength=sqrt(pow(local.requiredPosY.front()-local.robotY,2)+pow(local.requiredPosX.front()-local.robotX,2));
            }
           //recalculation of setpointAngle to angle from 0 to 360
            if(local.setpointAngle<0)
            {
                local.setpointAngle_0_360= 360+local.setpointAngle;
            }
            else{local.setpointAngle_0_360=local.setpointAngle;}


           //rotation speed control
            if(abs(local.setpointAngle_0_360-local.gyroAngle_0_360/100.0)>local.deadZone1Angle){

                local.rotating=true;
                local.translating=false;

               //this is recalculation to make the robot rotate a shorter path
                if(local.gyroAngle_180_180/100-local.setpointAngle>-180.0&&local.gyroAngle_180_180/100-local.setpointAngle<0.0){
                    local.outputAngleAction = local.P_reg_Angle.calculate(local.setpointAngle, robotdata.GyroAngle/100.0);
                }
                else if(local.setpointAngle_0_360-local.gyroAngle_0_360/100.0<=180){
                    local.outputAngleAction = local.P_reg_Angle.calculate(local.setpointAngle_0_360, local.gyroAngle_0_360/100.0);
                }
                else if((local.setpointAngle_0_360-local.gyroAngle_0_360/100.0)>180){
                    local.outputAngleAction = local.P_reg_Angle.calculate(local.setpointAngle, robotdata.GyroAngle/100.0);
                }



               //when is start of rotate the output from P angle regulator is limited by ramp
                if(local.startOfRotate==true){
                    local.outputAngleAction=ramp(local.outputAngleAction,0.1,&local.startOfRotate);
                    turn_around(local.outputAngleAction);
                }
               //when ramp ends then turning and deceleration speed is fully on P angle regulator
                else{
                    turn_around(local.outputAngleAction);
                }
            }


            //translation speed control
            if(abs(local.setpointAngle_0_360-local.gyroAngle_0_360/100.0)<=local.deadZone1Angle){

                local.outputLenAction = local.P_reg_Length.calculate(local.setpointLength, local.tr_dist_fr_lastP)*100.0;

               //when is start of translation the output from P translate regulator is limited by ramp
                if(local.startOfTranslate==true)
                {
                    local.outputLenAction=ramp(local.outputLenAction,10,&local.startOfTranslate);
                    go(local.outputLenAction);
                }
               //when ramp ends then translation and deceleration speed is fully on P translate regulator
                else
                {
                    go(local.outputLenAction);
                }
               local. rotating=false;
                local.translating=true;
            }
        }

    }
local.prevValGyro=robotdata.GyroAngle;

if(datacounter%5)
{
    emit uiValuesChangedR(local.robotX,local.robotY);
}
datacounter++;
}


/**
    Returns the gradually incrementing speed.
    @param speed. The actual speed to which it should increment.
    @param inc. Increment by which the speed is gradually increased.
    @param *start. When start is true the ramp begin and when ramp end then the start variable will change on false.

    @return The gradually incrementing speed.
*/
double MainWindow::ramp(double speed,double inc,bool *start)
{
    if(*start==true){
        double s=local.incPer;

        if(speed>=0){
            local.incPer=local.incPer+inc;
        }
        else{local.incPer=local.incPer-inc;}

        if(abs(s)>abs(speed))
        {
           local.incPer=0;
           *start=false;
           return speed;
        }
        return s;
    }
    else{
        return false;
    }

}



/**
    Returns the shortest distance from point do line.
    @param point_x,point_y. Are the X,Y position of point.
    @param line_x1,line_y1. Are the X,Y position of first point of line.
    @param line_x2,line_y2. Are the X,Y position of second point of line.

    @return The shortest distance from point do line.
*/
float MainWindow::distFromPointToLine(float point_x, float point_y, float line_x1, float line_y1, float line_x2, float line_y2)
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

double MainWindow::euclidDist(double x1,double y1,double x2,double y2)
{
   return sqrt(pow((x2-x1),2)+pow((y2-y1),2));
}

Point2f vect2d(cv::Point2f p1, Point2f p2) {
    Point2f temp;
    temp.x=(p2.x - p1.x);
    temp.y=(-1 * (p2.y - p1.y));
    return temp;}

/**
    Returns if point is in the rectangle.
    @param A,B,C,D. The points with(X,Y) coordinates of rectangle.
    @param m. The point which is checked if it is in the rectangle.

    @return If point is in the rectangle.
*/
bool pointInRectangle(Point2f A, Point2f B, Point2f C, Point2f D, Point2f m ) {
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

// funkcia local laser je naspracovanie dat z lasera(zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
int MainWindow::locallaser(LaserMeasurement &laserData)
{

    local.fusionPoints.clear();
    local.prekazka=false;
    if(MODE==2){
//        memset(local.robotMap,0, sizeof(short)*120*120);
        for (int i=0; i< 120;i++) //This variable is for each row below the x
        {
            for (int j=0; j<120;j++)
            {
                local.robotMap[i][j]=0;
            }
        }
    }

    for(int k=0;k<laserData.numberOfScans;k++){

        double dist=laserData.Data[k].scanDistance/1000.0;
        local.sensorDist[k]=dist;
        if(dist<0.14)
            continue;

       //get laser global points
        double xOfPoint=(dist*cos((360.0-laserData.Data[k].scanAngle+local.gyroAngle_180_180/100.0)*PI/180.0))+local.robotX;
        double yOfPoint=(dist*sin((360.0-laserData.Data[k].scanAngle+local.gyroAngle_180_180/100.0)*PI/180.0))+local.robotY;
        //insert points to map
        if(local.rotating==false&&dist<2.5&&MODE==3||MODE==2){
            local.robotMap[(int)round((xOfPoint/0.1)+60.0)][(int)round((yOfPoint/0.1)+60.0)]=1;
            local.robotStartCellX=(local.robotX*10.0)+60.0;
            local.robotStartCellY=(local.robotX*10.0)+60.0;

          //wideing of obstacles
            for(int i=-2;i<=2;i++){
                for(int j=-2;j<=2;j++){
                    local.robotMapWide[((int)round((xOfPoint/0.1)+60.0))+i][((int)round((yOfPoint/0.1)+60.0))+j]=1;
                }
            }
        }
        if(MODE==2){


        }



       //finding out if there is an obstacle in the way
        Point2f ptToCheck(xOfPoint, yOfPoint);
        if(!local.checkRequiredPosX.empty()&&(MODE==2||MODE==3)){
            Point2f midpoint(local.robotX+   ((local.checkRequiredPosX.front()-local.robotX)/2),local.robotY+    ((local.checkRequiredPosY.front()-local.robotY)/2));
           //create zone between robot position and required position
            RotatedRect zoneFromRobotToReqPos ( midpoint,Size2f(euclidDist(local.robotX,local.robotY,local.checkRequiredPosX.front(),local.checkRequiredPosY.front()),0.4), atan2(local.checkRequiredPosY.front() - local.robotY, local.checkRequiredPosX.front() - local.robotX)*180/PI);
            Point2f cornerPointsOfZone[4];
           //checking if some of laser points is in zone
            zoneFromRobotToReqPos.points(cornerPointsOfZone);
            if(pointInRectangle(cornerPointsOfZone[1],cornerPointsOfZone[0],cornerPointsOfZone[3],cornerPointsOfZone[2],ptToCheck)&&abs(ptToCheck.x)>0&&abs(ptToCheck.y)>0){
                local.prekazka=true;
                local.obstacleindex=k;
            }
        }


    }


   //find left and right points of obstacle
   if(local.prekazka==true&&MODE==2){

        for(int x=local.obstacleindex;x<=local.obstacleindex+laserData.numberOfScans/2;x++){
            if(x>=laserData.numberOfScans){
                x=x-laserData.numberOfScans+1;
            }

            double dist=laserData.Data[x].scanDistance/1000.0;
            double xOfPoint2=(dist*cos((360.0-laserData.Data[x].scanAngle+local.gyroAngle_0_360/100.0)*PI/180.0))+local.robotX;
            double yOfPoint2=(dist*sin((360.0-laserData.Data[x].scanAngle+local.gyroAngle_0_360/100.0)*PI/180.0))+local.robotY;

           if(((abs(laserData.Data[x+1].scanDistance/1000-laserData.Data[x].scanDistance/1000)>0.4) ) )
            {
                double shiftDist=laserData.Data[x].scanDistance/1000+0.5;
                double shiftAngle=atan2( 0.5,shiftDist) * 180 / PI;

                double xOfPoint=(shiftDist*cos((360.0-laserData.Data[x].scanAngle+shiftAngle+local.gyroAngle_0_360/100)*PI/180.0))+local.robotX;
                double yOfPoint=(shiftDist*sin((360.0-laserData.Data[x].scanAngle+shiftAngle+local.gyroAngle_0_360/100)*PI/180.0))+local.robotY;
                local.shortestShiftedLeftWay=(euclidDist(xOfPoint,yOfPoint,local.robotX,local.robotY)+euclidDist(xOfPoint,yOfPoint,local.checkRequiredPosX.front(),local.checkRequiredPosY.front()) );
                local.shortestShiftedLeftX=xOfPoint;
                local.shortestShiftedLeftY=yOfPoint;
                local.shortestLeftX=xOfPoint2;
                local.shortestLeftY=yOfPoint2;
                break;
            }

        }



        for(int x=local.obstacleindex;x>=local.obstacleindex-laserData.numberOfScans/2;x--){
            if(x<0){
                x=laserData.numberOfScans+1+x;
            }

            double dist=laserData.Data[x].scanDistance/1000.0;
            double xOfPoint2=(dist*cos((360.0-laserData.Data[x].scanAngle+local.gyroAngle_0_360/100.0)*PI/180.0))+local.robotX;
            double yOfPoint2=(dist*sin((360.0-laserData.Data[x].scanAngle+local.gyroAngle_0_360/100.0)*PI/180.0))+local.robotY;

           if(((abs(laserData.Data[x].scanDistance/1000-laserData.Data[x-1].scanDistance/1000)>0.4) ) )
            {
                double shiftDist=laserData.Data[x].scanDistance/1000+0.5;
                double shiftAngle=atan2( 0.5,shiftDist) * 180 / PI;

                double xOfPoint=(shiftDist*cos((360.0-laserData.Data[x].scanAngle-shiftAngle+local.gyroAngle_0_360/100)*PI/180.0))+local.robotX;
                double yOfPoint=(shiftDist*sin((360.0-laserData.Data[x].scanAngle-shiftAngle+local.gyroAngle_0_360/100)*PI/180.0))+local.robotY;
                local.shortestShiftedRightWay=(euclidDist(xOfPoint,yOfPoint,local.robotX,local.robotY)+euclidDist(xOfPoint,yOfPoint,local.checkRequiredPosX.front(),local.checkRequiredPosY.front()) );
                local.shortestShiftedRightX=xOfPoint;
                local.shortestShiftedRightY=yOfPoint;
                local.shortestRightX=xOfPoint2;
                local.shortestRightY=yOfPoint2;
                break;
            }

        }


        if(local.shortestShiftedLeftWay<local.shortestShiftedRightWay){
            local.finalShortestX=local.shortestShiftedLeftX;
            local.finalShortestY=local.shortestShiftedLeftY;
        }
        else{
            local.finalShortestX=local.shortestShiftedRightX;
            local.finalShortestY=local.shortestShiftedRightY;
        }
   }


//   cout<<local.shortestX2<<"  "<<local.shortestY2<<" "<<local.shortestX<<" "<<local.shortestY<<endl;




    if(local.prekazka&&MODE==3){
        local.requiredPosX.clear();
        local.requiredPosY.clear();
        local.endOfPositioning=true;
        ui->Warning_Prekazka_text->setVisible(true);

    }
    if(!local.prekazka&&!local.checkRequiredPosX.empty()&&MODE==3){
        local.requiredPosX.push_back(local.checkRequiredPosX.front());
        local.requiredPosY.push_back(local.checkRequiredPosY.front());
        local.selectedPoints.push_back(make_tuple((local.checkRequiredPosX.front()*10)+60,(local.checkRequiredPosY.front()*10)+60));

        local.checkRequiredPosX.pop_front();
        local.checkRequiredPosY.pop_front();

        local.endOfPositioning=false;
    }
    if(local.prekazka&&!local.selectedPoints.empty()&&MODE!=2){
        local.selectedPoints.pop_back();
    }
    if(!local.checkRequiredPosX.empty()){
            local.selectedPoints.push_back(make_tuple((local.checkRequiredPosX.front()*10)+60,(local.checkRequiredPosY.front()*10)+60));
    }




    cout<<local.prekazka<<"xxxxxxxxxxxxxxx"<<endl;
    cout<<local.checkRequiredPosX.size()<<"     fefefe"<<endl;
    cout<<local.requiredPosX.size()<<"     fefefe"<<endl;
    cout<<local.mozes<<"     fefefe"<<endl;


    if(local.prekazka&&MODE==2&&local.mozes&&!local.checkRequiredPosX.empty()){
        if(local.requiredPosX.empty()){
            local.requiredPosX.push_front(local.checkRequiredPosX.front());
            local.requiredPosY.push_front(local.checkRequiredPosY.front());
        }
        local.selectedPoints.push_back(make_tuple((local.checkRequiredPosX.front()*10)+60,(local.checkRequiredPosY.front()*10)+60));

        local.requiredPosX.push_front(local.finalShortestX);
        local.requiredPosY.push_front(local.finalShortestY);
//        local.checkRequiredPosX.clear();
//        local.checkRequiredPosY.clear();
        cout<<"velmi dobre"<<endl;
        local.endOfPositioning=false;
        local.mozes=false;
    }


    for(int k=0;k<120;k++){
        for(int l=0;l<120;l++){
            if(local.robotMap[k][l]==1){
                if(local.maxMapX<k){local.maxMapX=k;}
                if(local.minMapX>k){local.minMapX=k;}
                if(local.maxMapY<l){local.maxMapY=l;}
                if(local.minMapY>l){local.minMapY=l;}            }
        }
    }



   //fusion points calculation
    local.maxdist=0;
    for(int k=0;k<paintLaserData.numberOfScans;k++){
        double dist=laserData.Data[k].scanDistance/1000.0;
       //finding the maximum distance which laser detect
        if(dist*10>local.maxdist){local.maxdist=dist*10;}

        if(paintLaserData.Data[k].scanAngle>=333||paintLaserData.Data[k].scanAngle<=27){

            double fuziaZ=paintLaserData.Data[k].scanDistance/10.0*cos((paintLaserData.Data[k].scanAngle)*PI/180.0);
            double fuziaX=paintLaserData.Data[k].scanDistance/10.0*sin((paintLaserData.Data[k].scanAngle)*PI/180.0);

            local.xObr=(robotPicture.cols/2.0)-((628.036*fuziaX)/fuziaZ);
            local.yObr=(robotPicture.rows/2.0)+((628.036*local.fuziaY)/fuziaZ);

           //according distance push fusion points with different color to vector,it is painting in paintEvent
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




//--autonomousrobot simuluje slucku robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad polohovy regulator, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
void MainWindow::autonomousrobot(TKobukiData &sens)
{

    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
}
//--autonomouslaser simuluje spracovanie dat z robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad sposob obchadzania prekazky, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
int MainWindow::autonomouslaser(LaserMeasurement &laserData)
{
//    cout<<laserData.Data[0].scanDistance/1000.0<<"vtvrvr"<<endl;

//    if(laserData.Data[0].scanDistance/1000.0<0.14)
//    {
//        cout<<"zastavil som"<<endl;
//        sendRobotCommand(ROBOT_STOP);
//        requiredPosX.clear();
//    }
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
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




    if(updateCameraPicture==1){
       //painting fusion points to camera image
        for(int k=0;k<local.fusionPoints.size();k++){

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
       //scale camera image to fit to main frame
        imgIn= QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);
        if(!imgIn.isNull()){
            imgIn.operator=(imgIn.scaled(rect.width(),rect.height(),Qt::KeepAspectRatio,Qt::TransformationMode()));
        }
        updateCameraPicture=0;

    }

    if(updateLaserPicture==1&&!imgIn.isNull()){

       //prepare image into which map will be painted
        local.mapImage=QImage(local.maxMapX-local.minMapX+1,local.maxMapY-local.minMapY+1, QImage::Format_ARGB32);
        local.mapImage.fill(QColor(255,255,255,0));
       //paint map to map image
        for(int k=local.minMapX;k<=local.maxMapX;k++){
            for(int l=local.minMapY;l<=local.maxMapY;l++){
                if(local.robotMap[k][l]==1){
                    local.mapImage.setPixelColor((int)(k-local.minMapX),(int)(l-local.minMapY),QColor(255,131,0,255));
                }
                if(local.robotMap2[k][l]==-1){
                    local.mapImage.setPixelColor((int)(k-local.minMapX),(int)(l-local.minMapY),QColor(255,0,0,255));
                }
                if(local.robotMap2[k][l]==2){
                    local.mapImage.setPixelColor((int)(k-local.minMapX),(int)(l-local.minMapY),QColor(255,255,255,255));
                }

            }
        }
       //finding ratios for scaling map correctly
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




    if(local.show_Map_or_Camera=="camera"&&!imgIn.isNull()){

        painter.drawImage(rect.x()+(rect.width()-imgIn.width())/2,rect.y(),imgIn);
        QRect mapBackground(10,10,ui->frame1->width()/2.5+20.0,ui->frame1->height()/2.5+20.0);
        painter.drawRect(mapBackground);
       //show small map in left up corner
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

        painter.setBrush((Qt::white));
       //draw robot in map
        painter.drawEllipse(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+20-1.5*local.pomer,  ui->frame1->height()/2.5-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)+20-1.5*local.pomer-local.pomerY,3.0*local.pomer,3.0*local.pomer);
       //if there is some required position then we paint it
        if(local.selectedPoints.size()>0){
            pero.setStyle(Qt::SolidLine);
            pero.setWidth(local.pomer/2);
            pero.setColor(Qt::white);
            painter.setPen(pero);
           //draw line from robot to required position
            painter.drawLine(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+20, ui->frame1->height()/2.5-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)+20-local.pomerY,((get<0>(local.selectedPoints.front())-local.minMapX)*local.pomer)+20 ,ui->frame1->height()/2.5-((get<1>(local.selectedPoints.front())-local.minMapY)*local.pomer)+20-local.pomerY);

            pero.setStyle(Qt::SolidLine);
            pero.setWidth(local.pomer/4);
            painter.setPen(pero);
            painter.setBrush((Qt::red));
           //draw point where required position is
            painter.drawEllipse(((get<0>(local.selectedPoints.front())-local.minMapX)*local.pomer)+20-0.75*local.pomer ,ui->frame1->height()/2.5-((get<1>(local.selectedPoints.front())-local.minMapY)*local.pomer)+20-0.75*local.pomer-local.pomerY,1.5*local.pomer,1.5*local.pomer);

        }
        pero.setStyle(Qt::SolidLine);
        pero.setWidth(local.pomer/4);
        pero.setColor(Qt::black);
        painter.setPen(pero);
        int y=sin(local.gyroAngle_180_180/100*PI/180)*1.5*local.pomer;
        int x=cos(local.gyroAngle_180_180/100*PI/180)*1.5*local.pomer;
       //draw rotating line on robot according rotation of robot
        painter.drawLine(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+20, ui->frame1->height()/2.5-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)+20-local.pomerY, ((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+20+x,ui->frame1->height()/2.5-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)+20-local.pomerY-y);

    }

    if(local.show_Map_or_Camera=="map"&&!imgIn.isNull())
    {
       //rescale map to main frame
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

        painter.setBrush((Qt::white));
       //draw robot in map
        painter.drawEllipse(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+80-1.5*local.pomer,  ui->frame1->height()-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)-80-1.5*local.pomer-local.pomerY,3.0*local.pomer,3.0*local.pomer);


        if(MODE==2&&!local.requiredPosX.empty()&&local.prekazka){
            painter.setBrush((Qt::green));
            painter.drawEllipse(((60.0+(local.shortestShiftedLeftX*10.0)-local.minMapX)*local.pomer)+80-1.0*local.pomer,  ui->frame1->height()-((60.0+(local.shortestShiftedLeftY *10.0)-local.minMapY)*local.pomer)-80-1.0*local.pomer-local.pomerY,2.0*local.pomer,2.0*local.pomer);
            painter.drawEllipse(((60.0+(local.shortestShiftedRightX*10.0)-local.minMapX)*local.pomer)+80-1.0*local.pomer,  ui->frame1->height()-((60.0+(local.shortestShiftedRightY *10.0)-local.minMapY)*local.pomer)-1.0*local.pomer-80-local.pomerY,2.0*local.pomer,2.0*local.pomer);
            painter.setBrush((Qt::blue));
            painter.drawEllipse(((60.0+(local.shortestLeftX*10.0)-local.minMapX)*local.pomer)+80-1.0*local.pomer,  ui->frame1->height()-((60.0+(local.shortestLeftY *10.0)-local.minMapY)*local.pomer)-80-1.0*local.pomer-local.pomerY,2.0*local.pomer,2.0*local.pomer);
            painter.drawEllipse(((60.0+(local.shortestRightX*10.0)-local.minMapX)*local.pomer)+80-1.0*local.pomer,  ui->frame1->height()-((60.0+(local.shortestRightY *10.0)-local.minMapY)*local.pomer)-1.0*local.pomer-80-local.pomerY,2.0*local.pomer,2.0*local.pomer);
        }

       //if there is some required position then we paint it
        if(local.selectedPoints.size()>0){
            pero.setStyle(Qt::SolidLine);
            pero.setWidth(local.pomer/2);
            pero.setColor(Qt::white);
            painter.setPen(pero);
           //draw line from robot to required position
            painter.drawLine(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+80, ui->frame1->height()-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)-80-local.pomerY,((get<0>(local.selectedPoints.front())-local.minMapX)*local.pomer)+80 ,ui->frame1->height()-((get<1>(local.selectedPoints.front())-local.minMapY)*local.pomer)-80-local.pomerY);

            pero.setStyle(Qt::SolidLine);
            pero.setWidth(local.pomer/4);
            painter.setPen(pero);
            painter.setBrush((Qt::red));
           //draw point where required position is
            painter.drawEllipse(((get<0>(local.selectedPoints.front())-local.minMapX)*local.pomer)+80-0.75*local.pomer ,ui->frame1->height()-((get<1>(local.selectedPoints.front())-local.minMapY)*local.pomer)-80-0.75*local.pomer-local.pomerY,1.5*local.pomer,1.5*local.pomer);

        }
        pero.setStyle(Qt::SolidLine);
        pero.setWidth(local.pomer/4);
        pero.setColor(Qt::black);
        painter.setPen(pero);
        int y=sin(local.gyroAngle_180_180/100*PI/180)*1.5*local.pomer;
        int x=cos(local.gyroAngle_180_180/100*PI/180)*1.5*local.pomer;
       //draw rotating line on robot according rotation of robot
        painter.drawLine(((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+80, ui->frame1->height()-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)-80-local.pomerY, ((60.0+(local.robotX*10.0)-local.minMapX)*local.pomer)+80+x,ui->frame1->height()-((60.0+(local.robotY*10.0)-local.minMapY)*local.pomer)-80-local.pomerY-y);

    }






    if(showSkeleton==false&&ui->widget->isVisible()){
            ui->widget->setVisible(false);
    }


    //-------------------------------------------------------Drawing robot with sensors in sensor window---------------------------------------------------------

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
   //drawing robot in senzor window
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


  //drawing sensors

   //senzors in front of robot
    if(local.sensorDist[0]<0.6&&local.sensorDist[0]>0.05){
         painter.drawArc(rect3,67*16,46*16);
    }
    if(local.sensorDist[0]<0.5&&local.sensorDist[0]>0.05){
        painter.drawArc(rect2,67*16,44*16);
    }
    if(local.sensorDist[0]<0.4&&local.sensorDist[0]>0.05){
        painter.drawArc(rect1,69*16,42*16);
    }


   //left senzors in front of robot
    if(local.sensorDist[60]<0.6&&local.sensorDist[60]>0.05){
        painter.drawArc(rect3,127*16,47*16);
    }
    if(local.sensorDist[60]<0.5&&local.sensorDist[60]>0.05){
        painter.drawArc(rect2,127*16,45*16);
    }
    if(local.sensorDist[60]<0.4&&local.sensorDist[60]>0.05){
        painter.drawArc(rect1,129*16,42*16);
    }

   //left senzors behind the robot
    if(local.sensorDist[80]<0.6&&local.sensorDist[80]>0.05){
        painter.drawArc(rect3,188*16,46*16);
    }
    if(local.sensorDist[80]<0.5&&local.sensorDist[80]>0.05){
        painter.drawArc(rect2,189*16,44*16);
    }
    if(local.sensorDist[80]<0.4&&local.sensorDist[80]>0.05){
        painter.drawArc(rect1,189*16,42*16);
    }

   //senzors behind the robot
    if(local.sensorDist[138]<0.6&&local.sensorDist[138]>0.05){
        painter.drawArc(rect3,248*16,46*16);
    }
    if(local.sensorDist[138]<0.5&&local.sensorDist[138]>0.05){
        painter.drawArc(rect2,249*16,44*16);
    }
    if(local.sensorDist[138]<0.4&&local.sensorDist[138]>0.05){
        painter.drawArc(rect1,249*16,42*16);
    }

   //right senzors behind the robot
    if(local.sensorDist[198]<0.6&&local.sensorDist[198]>0.05){
        painter.drawArc(rect3,307*16,46*16);
    }
    if(local.sensorDist[198]<0.5&&local.sensorDist[198]>0.05){
        painter.drawArc(rect2,308*16,44*16);
    }
    if(local.sensorDist[198]<0.4&&local.sensorDist[198]>0.05){
        painter.drawArc(rect1,309*16,42*16);
    }

   //right senzors in front of robot
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
void  MainWindow::setUiValuesR(double robotX,double robotY)
{
     ui->Xrobot->setText(QString::number(ceil(robotX * 100.0) / 100.0));
     ui->Yrobot->setText(QString::number(ceil(robotY * 100.0) / 100.0));
}



void MainWindow::printMapToFile()
{

ofstream mapFile;
ofstream mapFileWide;

mapFile.open("example35.txt");
mapFileWide.open("wideMap.txt");

//if(mapFile)
//{
//cout<<"cannot file";}

    for (int i=120; i> 0;i--) //This variable is for each row below the x
    {
        for (int j=0; j<120;j++)
        {
            if(local.robotMap[j][i]==1){
                mapFile << "*";
            }else{
                mapFile << "-";
            }

            if(local.robotMapWide[j][i]==1){
                mapFileWide << "*";
            }else{
                mapFileWide << "-";
            }

        }
        mapFile<<std::endl;
        mapFileWide<<std::endl;

    }
    mapFile.close();
    mapFileWide.close();

}

void MainWindow::on_pushButton_12_clicked()
{
    if(!ui->lineEdit_X->text().isEmpty()&&!ui->lineEdit_Y->text().isEmpty()&&!local.prekazka)
    {
        if(MODE==2){
            local.checkRequiredPosX.push_back(ui->lineEdit_X->text().toDouble());
            local.checkRequiredPosY.push_back(ui->lineEdit_Y->text().toDouble());
            local.mozes=true;

        }
        if(MODE==3){
            local.checkRequiredPosX.push_back(ui->lineEdit_X->text().toDouble());
            local.checkRequiredPosY.push_back(ui->lineEdit_Y->text().toDouble());
            local.endOfPositioning=false;
        }

        if(MODE==4){
            local.Xnavigate=((ui->lineEdit_X->text().toDouble()*10)+60.0);
            local.Ynavigate=((ui->lineEdit_Y->text().toDouble()*10)+60.0);
            navigate_to_selected_point((int)local.Xnavigate,(int)local.Ynavigate);
        }
    }
 //    navigate_to_selected_point(int Xbunka,int Ybunka);
}

void MainWindow::on_cancelWarning_clicked(bool checked)
{
    if(local.prekazka&&ui->Warning_Prekazka_text->isVisible()){
        ui->Warning_Prekazka_text->setVisible(false);
        local.checkRequiredPosX.clear();
        local.checkRequiredPosY.clear();
        local.requiredPosX.clear();
        local.requiredPosY.clear();
        local.selectedPoints.clear();

        local.prekazka=false;
    }
}



bool MainWindow::navigate_to_selected_point(int Xcell,int Ycell)
{
    int XStart=(int)(local.robotX*10.0)+60.0;
    int YStart=(int)(local.robotY*10.0)+60.0;

    string direction="";
    string previousDirection=" ";

    int Xcandinate=0;
    int Ycandinate=0;

    int inc=0;
    deque< tuple <int,int,int> > mapNav;
    int robotMapWide_Navigate[120][120]={0};

    std::copy(&local.robotMapWide[0][0], &local.robotMapWide[0][0]+120*120,&robotMapWide_Navigate[0][0]);

    robotMapWide_Navigate[Xcell][Ycell]=2;

    mapNav.push_back(make_tuple(Xcell+1,Ycell,3));
    mapNav.push_back(make_tuple(Xcell-1,Ycell,3));
    mapNav.push_back(make_tuple(Xcell,Ycell+1,3));
    mapNav.push_back(make_tuple(Xcell,Ycell-1,3));


   //Flood fill algorithm
    do{
        Xcandinate=get<0>(mapNav.front());
        Ycandinate=get<1>(mapNav.front());

        if (robotMapWide_Navigate[Xcandinate][Ycandinate]==0){
           //Writing index to map if a candidate is 0
            robotMapWide_Navigate[Xcandinate][Ycandinate]=get<2>(mapNav.front());

           //adding neighbors with index as candidates
            if(Xcandinate+1<=120    &&  Ycandinate<=120    &&  Ycandinate>=0){
                mapNav.push_back(make_tuple(Xcandinate+1,Ycandinate,get<2>(mapNav.front())+1));
            }
            if(Xcandinate-1>0   &&  Ycandinate<=120    &&   Ycandinate>=0){
                mapNav.push_back(make_tuple(Xcandinate-1,Ycandinate,get<2>(mapNav.front())+1));
            }
            if(Ycandinate+1<=120    &&  Xcandinate<=120    &&  Xcandinate>=0){
                mapNav.push_back(make_tuple(Xcandinate,Ycandinate+1,get<2>(mapNav.front())+1));
            }
            if(Ycandinate-1>0   &&  Xcandinate<=120    &&   Xcandinate>=0){
                mapNav.push_back(make_tuple(Xcandinate,Ycandinate-1,get<2>(mapNav.front())+1));
            }

        }
        inc=get<2>(mapNav.front())+1;
        mapNav.pop_front();


    }while (!(Xcandinate==XStart&&Ycandinate==YStart)   &&  inc<120);


    if(inc==120){return false;}



   //find way from start to end
    if(inc<120){
        Xcandinate=XStart;
        Ycandinate=YStart;
        int Xcac=XStart;
        int Ycac=YStart;
        inc=inc-2;
        do{

            if(robotMapWide_Navigate[Xcandinate+1][Ycandinate]==inc){
                Xcac=Xcandinate;
                Ycac=Ycandinate;
                Xcandinate=Xcandinate+1;
                inc=inc-1;
                direction="right";

            }
            else if(robotMapWide_Navigate[Xcandinate-1][Ycandinate]==inc){

                Xcac=Xcandinate;
                Ycac=Ycandinate;
                Xcandinate=Xcandinate-1;
                inc=inc-1;
                direction="left";

            }
            else if(robotMapWide_Navigate[Xcandinate][Ycandinate+1]==inc){
                Xcac=Xcandinate;
                Ycac=Ycandinate;
                Ycandinate=Ycandinate+1;
                inc=inc-1;
                direction="up";

            }
            else if(robotMapWide_Navigate[Xcandinate][Ycandinate-1]==inc){
                Xcac=Xcandinate;
                Ycac=Ycandinate;

                Ycandinate=Ycandinate-1;

                inc=inc-1;
                direction="down";

            }

            if (previousDirection!=direction){

                previousDirection=direction;
                robotMapWide_Navigate[Xcac][Ycac]=-3;
                local.requiredPosX.push_back((double)((Xcac-60.0)/10.0));
                local.requiredPosY.push_back(double((Ycac-60.0)/10.0));
            }
            else if(robotMapWide_Navigate[Xcac][Ycac]!=-2){robotMapWide_Navigate[Xcac][Ycac]=-1;}


        }while (!(inc==1));

        robotMapWide_Navigate[XStart][YStart]=-2;
        local.requiredPosX.push_back((Xcell-60.0)/10.0);
        local.requiredPosY.push_back((Ycell-60.0)/10.0);
        local.requiredPosX.pop_front();
        local.requiredPosY.pop_front();
        local.endOfPositioning=false;


        std::copy(&local.robotMap[0][0], &local.robotMap[0][0]+120*120,&local.robotMap2[0][0]);
        ofstream mapFile;
        mapFile.open("indexes.txt");

        if(mapFile){
        cout<<"cannot file";}

            for (int i=120; i> 0;i--){
                for (int j=0; j<120;j++){
                    if(robotMapWide_Navigate[i][j]==-1||robotMapWide_Navigate[i][j]==-3){
                        local.robotMap2[i][j]=-1;
                    }
                    if(robotMapWide_Navigate[i][j]==2){
                        local.robotMap2[i][j]=2;
                    }
                    if(robotMapWide_Navigate[j][i]==0){
                        mapFile << "    ";
                    }
                    else if(robotMapWide_Navigate[j][i]<10&&robotMapWide_Navigate[j][i]>0){
                        mapFile << " "<<robotMapWide_Navigate[j][i]<<"  ";
                    }
                    else{
                        mapFile << " "<<robotMapWide_Navigate[j][i]<<" ";

                    }
                }
                mapFile<<std::endl;
            }
            mapFile.close();

        return true;
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
    connect(this,SIGNAL(uiValuesChangedR(double,double)),this,SLOT(setUiValuesR(double,double)));
    if(MODE==3){
        local.P_reg_Length.setmax(2);
        local.P_reg_Length.setmin(-2);

    }
    else{
        local.P_reg_Length.setmax(3);
        local.P_reg_Length.setmin(-3);

    }
    if(MODE==4){
        string line;
          ifstream myfile ("MapForLoad.txt");
          if (true){
              int x=0,y=120;

              while ( getline(myfile, line) )
                 {
                  cout<< line;

                      for (int i = 0; i < line.length(); i++) {

                              // Print current character
                              if(line[i]=='*'){
                                  local.robotMap[i][y]=1;
                                   cout<<i<<"vew"<<y<<endl;

                                  for(int u=-2;u<=2;u++){
                                      for(int j=-2;j<=2;j++){
                                          local.robotMapWide[i+u][y+j]=1;
                                      }
                                  }
                              }
                     }
                     y=y-1;
                 }



          }
          else{ cout << "Unable to open file";}
          printMapToFile();
    }
    if(MODE==2){
        string line;
        ifstream myfile ("MapForLoadUloha2.txt");
        if (true){
          int x=0,y=120;

          while ( getline(myfile, line) )
             {
              cout<< line;

                  for (int i = 0; i < line.length(); i++) {

                          // Print current character
                          if(line[i]=='*'){
                              local.robotMap[i][y]=1;
                               cout<<i<<"vew"<<y<<endl;

                              for(int u=-2;u<=2;u++){
                                  for(int j=-2;j<=2;j++){
                                      local.robotMapWide[i+u][y+j]=1;
                                  }
                              }
                          }
                 }
                 y=y-1;
             }



        }
        else{ cout << "Unable to open file";}
        printMapToFile();
    }


    local.robotX=0;
    local.robotY=0;

    showCamera=true;
    showLidar=true;
    showSkeleton=false;
    applyDelay=false;
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

//            applyDelay==true
            if(false)
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















