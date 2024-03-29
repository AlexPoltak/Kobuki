#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QTimer"
#include "QPainter"
#include "math.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include <QCoreApplication>
#include <QtConcurrent/QtConcurrent>
#include <deque>
#include <fstream>
#include <thread>
#include <chrono>

using namespace cv;
using namespace std;


std::deque<double> requiredPosX={};
std::deque<double> requiredPosY={};

std::deque<double> checkRequiredPosX={};
std::deque<double> checkRequiredPosY={};

//tuple(X OF Points,Y of points,color of points,size of points)
vector< tuple <double,double,string,double> > fusionPoints;
vector< tuple <double,double,string,double> > laserpoints;
vector< tuple <double,double,string,double> > mapPoints;

double sensorDist[277];
double did[277]={0};
double did2[277]={0};



double deadZone1Angle=0.5;
double deadZone2Angle=2;
double deadZoneTranslate=0.05;
double deadZoneToRequiredPos=0.04;


std::vector<int> dataER;
std::vector<int> dataEL;
std::vector<int> dataG;

int endOfPositioning=true;
bool startOfTranslate = true;
bool startOfRotate = true;
bool mapping=false;
double tr_dist_of_RW=0;
double tr_dist_of_LW=0;
double tr_dist=0;

//traveled distance from last point where was calculate setpoint
long double tr_dist_fr_lastP=0;
double startX=0;
double startY=0;

int is_overflow=0;
int is_overflowG=0;

long double tickToMeter = 0.000085292090497737556558; // [m/tick]

double setpointAngle=0;
double setpointAngle_0_360=0;
double setpointLength=0;

double outputAngleAction=0;
double outputLenAction=0;

PID P_reg_Length = PID(0.1, 3, -3, 12, 0, 0);
PID P_reg_Angle = PID(0.1, 3.14159, -3.14159, 0.2, 0, 0);
double xObr;
double yObr;


double xPMin=100000000;
double xPMax=0;
double yPMin=100000000;
double yPMax=0;




double endOflist=0;
int gyroAngle_0_360=0;
signed short gyroAngle_180_180=0;

signed short angleOnStart;
unsigned short prevValEncLeft=0;
unsigned short prevValEncRight=0;
short prevValGyro=0;


double fuziaY=6;

double imageWidth=0;

bool rotating=false;
bool translating=true;



short robotMap [ 120 ][ 120 ];


double shortestX=1000000;
double shortestY=1000000;
double shortest=1000000;

int maxMapY=0;
int prevMaxY=0;
int minMapY=10000000;

int maxMapX=0;
int prevMaxX=0;

int minMapX=10000000;

bool prekazka=false;
double shortestWay;

string show_Map_or_Camera="camera";
int AsizeX=0;
int AsizeY=0;
int BsizeX=0;
int BsizeY=0;



double robotStartCellX=60;
double robotStartCellY=60;
int u=0;

double x1=0;
double x2=0;
double y11=0;
double y2=0;


double pocetBuniekX=0;
double pocetBuniekY=0;
double Ypixels=0;
double pomer=0;
double Xpixels=0;
int pomerY=0;
double maxdist=0;





double Xnavigate;
double Ynavigate;


bool showObstacleWarning=false;
bool canGo=true;
bool pointSelected=false;
bool stoj=false;
bool dobre=true;
bool mozes=false;
double incPer=0;

bool centralSTOP=false;
QImage mapImage;
QImage camWindow;

vector< tuple <double,double> > selectedPoints;




bool stoped=false;
double countTraveledDistance(unsigned short encoder,std::vector<int>& data,bool print)
{

    data.push_back(encoder);
//    if(data.back()!=data.front()&&print==true){
//        cout<< encoder<<endl;
//        cout<<""<<endl;
//    }
    double traveled_Dist=0;
    is_overflow=data.back()-data.front();

    if(is_overflow<overflow_up){
       traveled_Dist= (tickToMeter*(data.back()-data.front()+65536));
       cout <<  "up" << endl;
    }

    else if(is_overflow>overflow_down){
       traveled_Dist= (tickToMeter*(data.back()-data.front()-65536));
       cout <<  "down" << endl;
    }

    else{
        traveled_Dist=(tickToMeter*(data.back()-data.front()));

    }
    data.clear();
    data.push_back(encoder);
    return traveled_Dist;
}

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
void MainWindow::localrobot(TKobukiData &robotdata)
{

    if(prvyStart)
    {
        angleOnStart=robotdata.GyroAngle;
        prevValEncLeft=robotdata.EncoderLeft;
        prevValEncRight=robotdata.EncoderRight;
        prevValGyro=robotdata.GyroAngle;

        prvyStart=false;

    }
    if(!centralSTOP){
       //overflow of left wheel
        is_overflow=robotdata.EncoderLeft-prevValEncLeft;
        if(is_overflow<overflow_up){
           tr_dist_of_LW= (tickToMeter*(robotdata.EncoderLeft-prevValEncLeft+65536));
           cout <<  "upL" << endl;
        }

        else if(is_overflow>overflow_down){
           tr_dist_of_LW= (tickToMeter*(robotdata.EncoderLeft-prevValEncLeft-65536));
           cout <<  "downL" << endl;
        }
        else{
            tr_dist_of_LW=(tickToMeter*(robotdata.EncoderLeft-prevValEncLeft));
        }
        prevValEncLeft=robotdata.EncoderLeft;


       //overflow of right wheel
        is_overflow=robotdata.EncoderRight-prevValEncRight;
        if(is_overflow<overflow_up){
           tr_dist_of_RW= (tickToMeter*(robotdata.EncoderRight-prevValEncRight+65536));
           cout <<  "upR" << endl;
        }

        else if(is_overflow>overflow_down){
           tr_dist_of_RW= (tickToMeter*(robotdata.EncoderRight-prevValEncRight-65536));
           cout <<  "downR" << endl;
        }
        else{
            tr_dist_of_RW=(tickToMeter*(robotdata.EncoderRight-prevValEncRight));
        }
        prevValEncRight=robotdata.EncoderRight;




    //    //calculation of traveled distance of both wheels from two last positions

        tr_dist=(tr_dist_of_RW+tr_dist_of_LW)/2;
        tr_dist_fr_lastP=tr_dist_fr_lastP+tr_dist;

        //recalculation of gyroangle to be 0 when code start and then it will be counting from this angle(+X is in front of robot,+Y is on left of robot)
        if(robotdata.GyroAngle-angleOnStart<-18000)
        {
            gyroAngle_180_180= robotdata.GyroAngle-angleOnStart+36000;
        }
        else if(robotdata.GyroAngle-angleOnStart>18000)
        {
            gyroAngle_180_180= robotdata.GyroAngle-angleOnStart-36000;
        }
        else
        {
            gyroAngle_180_180= robotdata.GyroAngle-angleOnStart;

        }

        //actual location of robot
        robotX=robotX+(tr_dist*cos((gyroAngle_180_180/100.0)* PI / 180.0));
        robotY=robotY+(tr_dist*sin((gyroAngle_180_180/100.0)* PI / 180.0));

        //finding of recalculation of Gyroangle to angle from 0 to 360
        if(gyroAngle_180_180<0){gyroAngle_0_360=gyroAngle_180_180+36000;}
        else{gyroAngle_0_360=gyroAngle_180_180;}
    //    cout<<robotX*100.0<<"X "<<robotY*100.0<<" Y "<<sens.GyroAngle<<"angle"<<endl;


    if(requiredPosX.size()>0&&prekazka==false&&!stoped){
        cout<<requiredPosX.front()<<"vrrerev"<<requiredPosY.front()<<"----------------------"<<endl;
        //if robot is in surrounding of current setpoints then set all parameters and set new required position
        if(euclidDist(robotX,robotY,requiredPosX.front(),requiredPosY.front())  <  deadZoneToRequiredPos)
        {
            startX=robotX;
            startY=robotY;
            startOfTranslate=true;
            startOfRotate=true;
            tr_dist_fr_lastP=0;
            requiredPosX.pop_front();
            requiredPosY.pop_front();
            checkRequiredPosX.clear();
            checkRequiredPosY.clear();
            dobre=true;
            stoj=false;

            if(selectedPoints.size()>0&&requiredPosX.empty()){
                selectedPoints.pop_back();
                pointSelected=false;
            }

            if((requiredPosX.empty()||requiredPosY.empty())&&endOfPositioning==false)
            {
                endOfPositioning=true;
                cout<<"end Of Positioning"<<endl;
                stop();
                printt();

            }
        }



        if(endOfPositioning==false){
         //Navigation
            //if distance of robot from trajectory is more than , or if angle of robot from setpoint angle is more than, then recalculate setpoints
            if(abs(distPointToLine(robotX,robotY,startX,startY,requiredPosX.front(),requiredPosY.front()))>deadZoneTranslate){tr_dist_fr_lastP=0;}
            if(abs(setpointAngle-gyroAngle_180_180/100.0)>deadZone2Angle){tr_dist_fr_lastP=0;}


            //finding setpoints for angle and distance to required position
            if(tr_dist_fr_lastP==0)
            {
                setpointAngle=(atan2(requiredPosY.front() -robotY,requiredPosX.front()-robotX)*180.0/PI);

                setpointLength=sqrt(pow(requiredPosY.front()-robotY,2)+pow(requiredPosX.front()-robotX,2));
            }

            if(setpointAngle<0)
            {
                setpointAngle_0_360= 360+setpointAngle;
            }
            else{setpointAngle_0_360=setpointAngle;}

            //rotation speed control
            if(abs(setpointAngle_0_360-gyroAngle_0_360/100.0)>deadZone1Angle)
            {
                cout<<setpointAngle_0_360<<" angle"<<gyroAngle_0_360<<endl;

                rotating=true;
                translating=false;

        //cout<<setpointAngle_0_360<<"  "<<gyroAngle_0_360<<""<<endl;
                //this is recalculation to make the robot rotate a shorter path
                 if(gyroAngle_180_180/100-setpointAngle>-180.0&&gyroAngle_180_180/100-setpointAngle<0.0)
                 {
                     outputAngleAction = P_reg_Angle.calculate(setpointAngle, robotdata.GyroAngle/100.0);
                 }
                 else if(setpointAngle_0_360-gyroAngle_0_360/100.0<=180)
                 {
                     outputAngleAction = P_reg_Angle.calculate(setpointAngle_0_360, gyroAngle_0_360/100.0);
                 }
                 else if((setpointAngle_0_360-gyroAngle_0_360/100.0)>180)
                 {
                     outputAngleAction = P_reg_Angle.calculate(setpointAngle, robotdata.GyroAngle/100.0);
                 }


    //            when is start of rotate the output from P angle regulator is limited by ramp
                if(startOfRotate==true)
                {
                    outputAngleAction=ramp(outputAngleAction,0.1,&startOfRotate);
                    turn_around(outputAngleAction);
                    cout<<outputAngleAction<<"rotationspeed1"<<endl;
                }
    //            else turning and deceleration is fully on P angle regulator
                else
                {
                    turn_around(outputAngleAction);
                    cout<<outputAngleAction<<"rotationspeed2"<<endl;
                }
            }


            //translation speed control
            if(abs(setpointAngle_0_360-gyroAngle_0_360/100.0)<=deadZone1Angle){


        //        cout<<rotating<<endl;
                //
                outputLenAction = P_reg_Length.calculate(setpointLength, tr_dist_fr_lastP)*100.0;

                //when is start of translation the output from P translate regulator is limited by ramp
                if(startOfTranslate==true)
                {
                    outputLenAction=ramp(outputLenAction,10,&startOfTranslate);
                    go(outputLenAction);
    //                cout<<outputLenAction<<"speed1"<<endl;
                }
                //else translation and deceleration is fully on P translate regulator
                else
                {
                    go(outputLenAction);
    //                cout<<outputLenAction<<"speed2"<<endl;
                }
                rotating=false;
                translating=true;

            }
        }

    //    if(dl%5==0)
    //    {
    //        ///toto je skaredy kod. rozumne je to posielat do ui cez signal slot..
    ////        emit uiValuesChanged(robotX,robotY,gyroAngle_0_360);

    //    }
    //    dl++;
    }
    prevValGyro=robotdata.GyroAngle;

    }
}

double MainWindow::ramp(double speed,double inc,bool *start)
{
    if(*start==true){
        double s=incPer;

        if(speed>=0){
            incPer=incPer+inc;
        }
        else{incPer=incPer-inc;}

        if(abs(s)>abs(speed))
        {
           incPer=0;
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
    paintThisLidar(laserData);

    fusionPoints.clear();
    shortestX=1000000;
    shortestY=1000000;
    shortest=1000000;
    maxMapY=0;
    minMapY=10000000;
    maxMapX=0;
    minMapX=10000000;

    for(int k=0;k<laserData.numberOfScans;k++)
    {   

        double dist=laserData.Data[k].scanDistance/1000.0;

        sensorDist[k]=dist;
        if(dist<0.14)
            continue;

        //get laser global points
        double xOfPoint=(dist*cos((360.0-laserData.Data[k].scanAngle+gyroAngle_0_360/100.0)*PI/180.0))+robotX;
        double yOfPoint=(dist*sin((360.0-laserData.Data[k].scanAngle+gyroAngle_0_360/100.0)*PI/180.0))+robotY;
        //insert points to map
        if(rotating==false&&dist<2.5){
            robotMap[(int)round((xOfPoint/0.1)+60.0)][(int)round((yOfPoint/0.1)+60.0)]=1;
            robotStartCellX=(robotX*10.0)+60.0;
            robotStartCellY=(robotX*10.0)+60.0;          
        }

        if(!checkRequiredPosX.empty()&&prekazka==false){
            Point2f pt(xOfPoint, yOfPoint);
            Point2f midpoint(robotX+   ((checkRequiredPosX.front()-robotX)/2),robotY+    ((checkRequiredPosY.front()-robotY)/2));
            RotatedRect rr1 ( midpoint,Size2f(euclidDist(robotX,robotY,checkRequiredPosX.front(),checkRequiredPosY.front()),0.4), atan2(checkRequiredPosY.front() - robotY, checkRequiredPosX.front() - robotX)*180/PI);
            Point2f vtx[4];

            rr1.points(vtx);
            if(pointInRectangle(vtx[1],vtx[0],vtx[3],vtx[2],pt)&&abs(pt.x)>0&&abs(pt.y)>0){
                prekazka=true;
                cout<<"prekazka"<<endl;
            }

        }

    }

    cout<<stoped<<"  "<<dobre<<"   "<<checkRequiredPosX.size()<<endl;

    if(stoped==true&&!checkRequiredPosX.empty()&&dobre){
        checkRequiredPosX.clear();
        checkRequiredPosY.clear();
        requiredPosX.clear();
        requiredPosY.clear();
        selectedPoints.clear();
        endOfPositioning=false;
        ui->Warning_Prekazka_text->setText("skoro som narazil");
        ui->Warning_Prekazka_text->setVisible(true);
        dobre=false;
        mozes=true;
    }

    if(!prekazka&&!checkRequiredPosX.empty()&&mozes){
        requiredPosX.push_back(checkRequiredPosX.front());
        requiredPosY.push_back(checkRequiredPosY.front());
        endOfPositioning=false;
        pointSelected=false;
        mozes=false;
    }

//    if(stoj==true&&!checkRequiredPosX.empty()&&euclidDist(robotX,robotY,checkRequiredPosX.front(),checkRequiredPosY.front())>0.3){dobre=true;stoj=false;}

    if(prekazka==true)
    {
        sendRobotCommand(ROBOT_STOP);
        ui->Warning_Prekazka_text->setText("V ceste je prekážka, naviguj na iné miesto");
        ui->Warning_Prekazka_text->setVisible(true);
        endOfPositioning=true;
        requiredPosX.clear();
        requiredPosY.clear();
        selectedPoints.clear();
        prekazka=false;

    }
//    if(!canGo&&stoj==true)
//    {
//        sendRobotCommand(ROBOT_STOP);
//        ui->Warning_Prekazka_text->setText("naburam");
//        ui->Warning_Prekazka_text->setVisible(true);
//        endOfPositioning=true;
//        requiredPosX.clear();
//        requiredPosY.clear();
//        selectedPoints.clear();
//        pointSelected=false;


//    }
//    if(!canGo&&!checkRequiredPosX.empty()){
//        cout<<"tututututututututututututu"<<endl;
//        sendRobotCommand(ROBOT_STOP);
//        endOfPositioning=true;
//        requiredPosX.clear();
//        requiredPosY.clear();
//        checkRequiredPosX.clear();
//        checkRequiredPosY.clear();
//        ui->Warning_Prekazka_text->setText("Takmer som nabural, naviguj inde");
//        ui->Warning_Prekazka_text->setVisible(true);
//        canGo=true;
//        selectedPoints.clear();

//    }
//    if(prekazka==false&&!checkRequiredPosX.empty()){
//        cout<<"----------------------------------------------------------"<<endl;
//        requiredPosX.push_back(checkRequiredPosX.front());
//        requiredPosY.push_back(checkRequiredPosY.front());

//        endOfPositioning=false;
//        pointSelected=false;

//    }
    if(prekazka==true&&selectedPoints.size()>0){
        selectedPoints.pop_back();
    }

//    if(prekazka==false&&checkrequiredPosX!=-1){

//        requiredPosX.push_back(checkrequiredPosX);
//        requiredPosY.push_back(checkrequiredPosY);
//        endOfPositioning=false;

//    }


//    if(prekazka==true&&endOfPositioning==false){
//        requiredPosX.push_front(shortestX);
//        requiredPosY.push_front(shortestY);
////        cout<<shortestX<<"        "<<shortestY<<" shortest  k= "<<endl;
//    }

    for(int k=0;k<120;k++){
        for(int l=0;l<120;l++){
            if(robotMap[k][l]==1){
                if(maxMapX<k){maxMapX=k;}
                if(minMapX>k){minMapX=k;}
                if(maxMapY<l){maxMapY=l;}
                if(minMapY>l){minMapY=l;}            }
        }
    }
    if(dl%2==0)
    {
        prevMaxY=maxMapY;
        prevMaxX=maxMapX;
    }
    dl++;



//    fusion points calculation
    maxdist=0;
    for(int k=0;k<paintLaserData.numberOfScans;k++){
        double dist=laserData.Data[k].scanDistance/1000.0;
        if(dist*10>maxdist){maxdist=dist*10;}

        if(paintLaserData.Data[k].scanAngle>=333||paintLaserData.Data[k].scanAngle<=27){

            double fuziaZ=paintLaserData.Data[k].scanDistance/10.0*cos((paintLaserData.Data[k].scanAngle)*PI/180.0);
            double fuziaX=paintLaserData.Data[k].scanDistance/10.0*sin((paintLaserData.Data[k].scanAngle)*PI/180.0);
//            xObr=ui->frame1->width()-((imgIn.width()/2.0)-((628.036*fuziaX)/fuziaZ));
//            yObr=(ui->frame1->height()/2.0)+((628.036*fuziaY)/fuziaZ);
            xObr=(robotPicture.cols/2.0)-((628.036*fuziaX)/fuziaZ);
            yObr=(robotPicture.rows/2.0)+((628.036*fuziaY)/fuziaZ);


            double dist=paintLaserData.Data[k].scanDistance/1000.0-0.3;
            if(dist<=0.1)
            {
                fusionPoints.push_back(make_tuple(xObr,yObr,"red",dist*10));
            }
            else if(dist>0.1&&dist<=0.2)
            {
                fusionPoints.push_back(make_tuple(xObr,yObr,"orange",dist*10));
            }
            else if(dist>0.2&&dist<=0.5)
            {
                fusionPoints.push_back(make_tuple(xObr,yObr,"yellow",dist*10));
            }
            else
            {
                fusionPoints.push_back(make_tuple(xObr,yObr,"gray",dist*10));
            }

        }
    }


    return -1;
}


//--autonomousrobot simuluje slucku robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad polohovy regulator, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
void MainWindow::autonomousrobot(TKobukiData &sens)
{












}
//--autonomouslaser simuluje spracovanie dat z robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad sposob obchadzania prekazky, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
int MainWindow::autonomouslaser(LaserMeasurement &laserData)
{

    for(int k=0;k<laserData.numberOfScans;k++)
    {
        if(laserData.Data[k].scanDistance/1000.0<0.3&&laserData.Data[k].scanDistance/1000.0>0.0)
        {
            sendRobotCommand(ROBOT_STOP);
            stoped=true;
        }
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
    show_Map_or_Camera="map";
    ui->showCam->setStyleSheet("color: white; background-color: rgba(0, 0, 0, 0.7); border-style: outset; border-width: 1px; border-color: beige");
    ui->showMap->setStyleSheet("color: black; background-color: white; border-style: outset; border-width: 1px; border-color: beige");

}

void MainWindow::on_showCam_clicked(bool checked)
{
    showCamera=true;
    show_Map_or_Camera="camera";
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

        for(int k=0;k<fusionPoints.size();k++)
            {
//            cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(fusionPoints[k])-3,get<1>(fusionPoints[k])+10), cv::Point(robotPicture.cols-get<0>(fusionPoints[k])+3,get<1>(fusionPoints[k])-10), Scalar(0,0,0), -1,LINE_4);

                if(get<2>(fusionPoints[k])=="red"){
                    cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(fusionPoints[k])-2,get<1>(fusionPoints[k])+abs(get<3>(fusionPoints[k])-maxdist)/2), cv::Point(robotPicture.cols-get<0>(fusionPoints[k])+2,get<1>(fusionPoints[k])-abs(get<3>(fusionPoints[k])-maxdist)/2), Scalar(0,0,255), -1,LINE_4);
                }
                if(get<2>(fusionPoints[k])=="orange"){
                    cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(fusionPoints[k])-2,get<1>(fusionPoints[k])+abs(get<3>(fusionPoints[k])-maxdist)/2), cv::Point(robotPicture.cols-get<0>(fusionPoints[k])+2,get<1>(fusionPoints[k])-abs(get<3>(fusionPoints[k])-maxdist)/2), Scalar(0,128,255), -1,LINE_4);
                }
                if(get<2>(fusionPoints[k])=="yellow"){
                    cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(fusionPoints[k])-2,get<1>(fusionPoints[k])+abs(get<3>(fusionPoints[k])-maxdist)/2), cv::Point(robotPicture.cols-get<0>(fusionPoints[k])+2,get<1>(fusionPoints[k])-abs(get<3>(fusionPoints[k])-maxdist)/2), Scalar(0,255,255), -1,LINE_4);
                }
                if(get<2>(fusionPoints[k])=="gray"){             
                    cv::rectangle(robotPicture, cv::Point(robotPicture.cols-get<0>(fusionPoints[k])-2,get<1>(fusionPoints[k])+abs(get<3>(fusionPoints[k])-maxdist)/2), cv::Point(robotPicture.cols-get<0>(fusionPoints[k])+2,get<1>(fusionPoints[k])-abs(get<3>(fusionPoints[k])-maxdist)/2), Scalar(128,128,128), -1,LINE_4);
                }
            }
        imgIn= QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);
        imgIn.operator=(imgIn.scaled(rect.width(),rect.height(),Qt::KeepAspectRatio,Qt::TransformationMode()));

        camWindow=QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);
        camWindow.operator=(camWindow.scaled(camWindowRect.width(),camWindowRect.height(),Qt::KeepAspectRatio,Qt::TransformationMode()));

        updateCameraPicture=0;

    }

    if(updateLaserPicture==1 )
    {

        mapImage=QImage(maxMapX-minMapX+1,maxMapY-minMapY+1, QImage::Format_ARGB32);
        mapImage.fill(QColor(255,255,255,0));

        for(int k=minMapX;k<=maxMapX;k++)
        {
            for(int l=minMapY;l<=maxMapY;l++)
            {
                if(robotMap[k][l]==1){
                    mapImage.setPixelColor((int)(k-minMapX),(int)(l-minMapY),QColor(255,131,0,255));
                }
            }
        }
        pocetBuniekX=maxMapX-minMapX;
        pocetBuniekY=maxMapY-minMapY;
        Ypixels=0;
        pomer=0;
        Xpixels=0;
        pomerY=0;


        pero.setStyle(Qt::SolidLine);
        pero.setWidth(3);
        pero.setColor(Qt::white);
        painter.setPen(pero);
        painter.setBrush(QColor(0,0,0,127));

        mapImage=mapImage.mirrored(0,1);

    }




    if(show_Map_or_Camera=="camera"){

        painter.drawImage(rect.x()+(rect.width()-imgIn.width())/2,rect.y(),imgIn);
        QRect mapBackground(10,10,ui->frame1->width()/2.5+20.0,ui->frame1->height()/2.5+20.0);
        painter.drawRect(mapBackground);
        painter.drawImage(20,20,mapImage.scaled(ui->frame1->width()/2.5,ui->frame1->height()/2.5,Qt::KeepAspectRatio,Qt::TransformationMode()));
        Ypixels=ui->frame1->height()/2.5;
        Xpixels=ui->frame1->width()/2.5;

        if((Xpixels/pocetBuniekX)<(Ypixels/pocetBuniekY)){
            pomer=(double)(Xpixels/pocetBuniekX);
            pomerY=(double)ui->frame1->height()/2.5-(pomer*pocetBuniekY);

        }
        else{
            pomer=(double)(Ypixels/pocetBuniekY);
        }

        double posunY=((60.0+(robotY*10.0)-minMapY)*pomer)-10.0;
        painter.setBrush((Qt::white));
        painter.drawEllipse(((60.0+(robotX*10.0)-minMapX)*pomer)+20-1.5*pomer,  ui->frame1->height()/2.5-((60.0+(robotY*10.0)-minMapY)*pomer)+20-1.5*pomer-pomerY,3.0*pomer,3.0*pomer);

        if(selectedPoints.size()>0){
            pero.setStyle(Qt::SolidLine);
            pero.setWidth(pomer/2);
            pero.setColor(Qt::white);
            painter.setPen(pero);

            painter.drawLine(((60.0+(robotX*10.0)-minMapX)*pomer)+20, ui->frame1->height()/2.5-((60.0+(robotY*10.0)-minMapY)*pomer)+20-pomerY,((get<0>(selectedPoints.front())-minMapX)*pomer)+20 ,ui->frame1->height()/2.5-((get<1>(selectedPoints.front())-minMapY)*pomer)+20-pomerY);

            pero.setStyle(Qt::SolidLine);
            pero.setWidth(pomer/4);
            painter.setPen(pero);
            painter.setBrush((Qt::red));
            painter.drawEllipse(((get<0>(selectedPoints.front())-minMapX)*pomer)+20-0.75*pomer ,ui->frame1->height()/2.5-((get<1>(selectedPoints.front())-minMapY)*pomer)+20-0.75*pomer-pomerY,1.5*pomer,1.5*pomer);

        }
        pero.setStyle(Qt::SolidLine);
        pero.setWidth(pomer/4);
        pero.setColor(Qt::black);
        painter.setPen(pero);
        int yyy=sin(gyroAngle_180_180/100*PI/180)*1.5*pomer;
        int xxx=cos(gyroAngle_180_180/100*PI/180)*1.5*pomer;

        painter.drawLine(((60.0+(robotX*10.0)-minMapX)*pomer)+20, ui->frame1->height()/2.5-((60.0+(robotY*10.0)-minMapY)*pomer)+20-pomerY, ((60.0+(robotX*10.0)-minMapX)*pomer)+20+xxx,ui->frame1->height()/2.5-((60.0+(robotY*10.0)-minMapY)*pomer)+20-pomerY-yyy);




    }

    if(show_Map_or_Camera=="map")
    {
//        fusionPoints.clear();
        painter.drawImage(80,80,mapImage.scaled(ui->frame1->width()-360,ui->frame1->height()-160,Qt::KeepAspectRatio,Qt::TransformationMode()));
        Ypixels=ui->frame1->height()-160;
        Xpixels=ui->frame1->width()-360;

        if((Xpixels/pocetBuniekX)<(Ypixels/pocetBuniekY)){
            pomer=(double)(Xpixels/pocetBuniekX);
            pomerY=(double)ui->frame1->height()-160-(pomer*pocetBuniekY);

        }
        else{
            pomer=(double)(Ypixels/pocetBuniekY);
        }

        double posunY=((60.0+(robotY*10.0)-minMapY)*pomer);
        double posunX=(((robotX*10.0))*pomer)+((60.0-minMapX)*pomer);
        painter.setBrush((Qt::white));
        painter.drawEllipse(((60.0+(robotX*10.0)-minMapX)*pomer)+80-1.5*pomer,  ui->frame1->height()-((60.0+(robotY*10.0)-minMapY)*pomer)-80-1.5*pomer-pomerY,3.0*pomer,3.0*pomer);



        if(selectedPoints.size()>0){
            pero.setStyle(Qt::SolidLine);
            pero.setWidth(pomer/2);
            pero.setColor(Qt::white);
            painter.setPen(pero);

            painter.drawLine(((60.0+(robotX*10.0)-minMapX)*pomer)+80, ui->frame1->height()-((60.0+(robotY*10.0)-minMapY)*pomer)-80-pomerY,((get<0>(selectedPoints.front())-minMapX)*pomer)+80 ,ui->frame1->height()-((get<1>(selectedPoints.front())-minMapY)*pomer)-80-pomerY);

            pero.setStyle(Qt::SolidLine);
            pero.setWidth(pomer/4);
            painter.setPen(pero);
            painter.setBrush((Qt::red));
            painter.drawEllipse(((get<0>(selectedPoints.front())-minMapX)*pomer)+80-0.75*pomer ,ui->frame1->height()-((get<1>(selectedPoints.front())-minMapY)*pomer)-80-0.75*pomer-pomerY,1.5*pomer,1.5*pomer);

        }
        pero.setStyle(Qt::SolidLine);
        pero.setWidth(pomer/4);
        pero.setColor(Qt::black);
        painter.setPen(pero);
        int yyy=sin(gyroAngle_180_180/100*PI/180)*1.5*pomer;
        int xxx=cos(gyroAngle_180_180/100*PI/180)*1.5*pomer;
        painter.drawLine(((60.0+(robotX*10.0)-minMapX)*pomer)+80, ui->frame1->height()-((60.0+(robotY*10.0)-minMapY)*pomer)-80-pomerY, ((60.0+(robotX*10.0)-minMapX)*pomer)+80+xxx,ui->frame1->height()-((60.0+(robotY*10.0)-minMapY)*pomer)-80-pomerY-yyy);

        painter.drawImage(camWindowRect.x()+(camWindowRect.width()-camWindow.width())/2,camWindowRect.y(),camWindow);

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

            if(sensorDist[0]<0.6&&sensorDist[0]>0.05){
                 painter.drawArc(rect3,67*16,46*16);
            }
            if(sensorDist[0]<0.5&&sensorDist[0]>0.05){
                painter.drawArc(rect2,67*16,44*16);
            }
            if(sensorDist[0]<0.4&&sensorDist[0]>0.05){
                painter.drawArc(rect1,69*16,42*16);
            }


//LH
            if(sensorDist[60]<0.6&&sensorDist[60]>0.05){
                painter.drawArc(rect3,127*16,47*16);
            }
            if(sensorDist[60]<0.5&&sensorDist[60]>0.05){
                painter.drawArc(rect2,127*16,45*16);
            }
            if(sensorDist[60]<0.4&&sensorDist[60]>0.05){
                painter.drawArc(rect1,129*16,42*16);
            }

//LD
            if(sensorDist[80]<0.6&&sensorDist[80]>0.05){
                painter.drawArc(rect3,188*16,46*16);
            }
            if(sensorDist[80]<0.5&&sensorDist[80]>0.05){
                painter.drawArc(rect2,189*16,44*16);
            }
            if(sensorDist[80]<0.4&&sensorDist[80]>0.05){
                painter.drawArc(rect1,189*16,42*16);
            }

//D

            if(sensorDist[138]<0.6&&sensorDist[138]>0.05){
                painter.drawArc(rect3,248*16,46*16);
            }
            if(sensorDist[138]<0.5&&sensorDist[138]>0.05){
                painter.drawArc(rect2,249*16,44*16);
            }
            if(sensorDist[138]<0.4&&sensorDist[138]>0.05){
                painter.drawArc(rect1,249*16,42*16);
            }


//PD

            if(sensorDist[198]<0.6&&sensorDist[198]>0.05){
                painter.drawArc(rect3,307*16,46*16);
            }
            if(sensorDist[198]<0.5&&sensorDist[198]>0.05){
                painter.drawArc(rect2,308*16,44*16);
            }
            if(sensorDist[198]<0.4&&sensorDist[198]>0.05){
                painter.drawArc(rect1,309*16,42*16);
            }

//PH

            if(sensorDist[218]<0.6&&sensorDist[218]>0.05){
                painter.drawArc(rect3,7*16,47*16);
            }
            if(sensorDist[218]<0.5&&sensorDist[218]>0.05){
                painter.drawArc(rect2,8*16,44*16);
            }
            if(sensorDist[218]<0.4&&sensorDist[218]>0.05){
                painter.drawArc(rect1,9*16,42*16);
            }

}



void MainWindow::mousePressEvent(QMouseEvent *event)
{
    QPoint lastPoint = event->pos();

    if (event->button() == Qt::LeftButton&&show_Map_or_Camera=="map"&&lastPoint.x()<(ui->frame1->width()-220)){

        if( ui->Warning_Prekazka_text->isVisible()&&centralSTOP==false)
        {
            ui->Warning_Prekazka_text->setText("V ceste je prekážka, naviguj na iné miesto");
            ui->Warning_Prekazka_text->setVisible(false);
            prekazka=false;

            if(!checkRequiredPosX.empty()){
                checkRequiredPosX.clear();
                checkRequiredPosY.clear();
            }

        }
        else if(checkRequiredPosX.empty()&&centralSTOP==false){

            Xnavigate=((lastPoint.x()-80.0)/pomer)+minMapX;
            Ynavigate=maxMapY-((lastPoint.y()-80.0)/pomer);

            checkRequiredPosX.push_back((Xnavigate-60.0)/10.0);
            checkRequiredPosY.push_back((Ynavigate-60.0)/10.0);
            selectedPoints.push_back(make_tuple(Xnavigate,Ynavigate));
            mozes=true;
        }
        else if(centralSTOP==true){
            ui->Warning_Prekazka_text->setText("Ak chces navigovat, musis vypnut central STOP");
            ui->Warning_Prekazka_text->setVisible(true);

        }



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
            if(robotMap[j][i]==1){
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
    if(!centralSTOP){
        ui->pushButton_12->setStyleSheet("color: #FFF; background-color: #f54242; border: 2px solid #f54242; border-radius: 2px;");
        requiredPosX.clear();
        requiredPosY.clear();
        selectedPoints.clear();
        endOfPositioning=true;
        stop();
        centralSTOP=true;
    }
    else{
        ui->pushButton_12->setStyleSheet("color: #f54242; background-color: #FFF; border: 2px solid #f54242; border-radius: 2px;");
        centralSTOP=false;
        ui->Warning_Prekazka_text->setVisible(false);


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


    robotX=0;
    robotY=0;
    robotFi=0;

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













