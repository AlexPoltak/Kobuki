#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#ifdef _WIN32
#include<windows.h>
#else
#include <termios.h>
#include <unistd.h>
#include "unistd.h"
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#endif
#include<iostream>

#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include<algorithm>
#include<chrono>
#include "CKobuki.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rplidar.h"
#include <QThread>
#include <QKeyEvent>
#include "regulator.h"
#include <tuple>
#include <deque>

#define ROBOT_VPRED 0x01
#define ROBOT_VZAD 0x02
#define ROBOT_VLAVO 0x04
#define ROBOT_VPRAVO 0x03
#define ROBOT_STOP 0x00
#define ROBOT_ARC 0x05

#define overflow_up -60000
#define overflow_down 60000

static const char *klby[]={{"left_wrist"},{"left_thumb_cmc"},{"left_thumb_mcp"},{"left_thumb_ip"},{"left_thumb_tip"},{"left_index_cmc"},{"left_index_mcp"},{"left_index_ip"},{"left_index_tip"},{"left_middle_cmc"},{"left_middle_mcp"},{"left_middle_ip"},{"left_middle_tip"},{"left_ring_cmc"},{"left_ring_mcp"},{"left_ring_ip"},{"left_ringy_tip"},{"left_pinky_cmc"},{"left_pink_mcp"},{"left_pink_ip"},{"left_pink_tip"},{"right_wrist"},{"right_thumb_cmc"},{"right_thumb_mcp"},{"right_thumb_ip"},{"right_thumb_tip"},{"right_index_cmc"},{"right_index_mcp"},{"right_index_ip"},{"right_index_tip"},{"right_middle_cmc"},{"right_middle_mcp"},{"right_middle_ip"},{"right_middle_tip"},{"right_ring_cmc"},{"right_ring_mcp"},{"right_ring_ip"},{"right_ringy_tip"},{"right_pinky_cmc"},{"right_pink_mcp"},{"right_pink_ip"},{"right_pink_tip"},{"nose"},{"left_eye_in"},{"left_eye"},{"left_eye_out"},{"right_eye_in"},{"right_eye"},{"right_eye_out"},{"left_ear"},{"right_ear"},{"mounth_left"},{"mounth_right"},{"left_shoulder"},{"right_shoulder"},{"left_elbow"},{"right_elbow"},{"left_wrist"},{"right_wrist"},{"left_pinky"},{"right_pinky"},{"left_index"},{"right_index"},{"left_thumb"},{"right_thumb"},{"left_hip"},{"right_hip"},{"left_knee"},{"right_knee"},{"left_ankle"},{"right_ankle"},{"left_heel"},{"righ_heel"},{"left+foot_index"},{"right_foot_index"}};
enum jointnames
{
    left_wrist,
   left_thumb_cmc,
   left_thumb_mcp,
   left_thumb_ip,
   left_thumb_tip,
   left_index_cmc,
   left_index_mcp,
   left_index_ip,
   left_index_tip,
   left_middle_cmc,
   left_middle_mcp,
   left_middle_ip,
   left_middle_tip,
   left_ring_cmc,
   left_ring_mcp,
   left_ring_ip,
   left_ringy_tip,
   left_pinky_cmc,
   left_pink_mcp,
   left_pink_ip,
   left_pink_tip,
   right_wrist,
   right_thumb_cmc,
   right_thumb_mcp,
   right_thumb_ip,
   right_thumb_tip,
   right_index_cmc,
   right_index_mcp,
   right_index_ip,
   right_index_tip,
   right_middle_cmc,
   right_middle_mcp,
   right_middle_ip,
   right_middle_tip,
   right_ring_cmc,
   right_ring_mcp,
   right_ring_ip,
   right_ringy_tip,
   right_pinky_cmc,
   right_pink_mcp,
   right_pink_ip,
   right_pink_tip,
   nose,left_eye_in,
   left_eye,
   left_eye_out,
   right_eye_in,
   right_eye,
   right_eye_out,
   left_ear,
   right_ear,
   mounth_left,
   mounth_right,
   left_shoulder,
   right_shoulder,
   left_elbow,
   right_elbow,
   left_wrist_glob,
   right_wrist_glob,
   left_pinky,
   right_pinky,
   left_index,
   right_index,
   left_thumb,
   right_thumb,
   left_hip,
   right_hip,
   left_knee,
   right_knee,
   left_ankle,
   right_ankle,
   left_heel,
   righ_heel,
   left_foot_index,
   right_foot_index
};
typedef struct
{
    double x;
    double y;
    double z;
}klb;

typedef struct
{
    klb joints[75];
}skeleton;

typedef struct
{
     std::chrono::steady_clock::time_point timestamp;
    int command;
    double speed;
    int radius;
}RobotCommand;

typedef struct
{
     std::chrono::time_point<std::chrono::steady_clock> timestamp;
    TKobukiData sens;
}RobotData;


typedef struct
{
    int commandType;//0 ziaden, 1 pohyb, 2 uhol
     int desiredDist;
     int actualDist;
     int desiredAngle;
     int actualAngle;
}AutonomousCommand;

typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    AutonomousCommand command;
}CommandVector;

typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    LaserMeasurement data;
}LidarVector;


typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    cv::Mat data;
}CameraVector;


struct local
{
    std::deque<double> requiredPosX={};
    std::deque<double> requiredPosY={};
    std::deque<double> checkRequiredPosX={};
    std::deque<double> checkRequiredPosY={};
    //tuple(X OF Points,Y of points,color of points,size of points)
    vector< tuple <double,double,string,double> > fusionPoints;
    vector< tuple <double,double,string,double> > laserpoints;
    vector< tuple <double,double,string,double> > mapPoints;
    double sensorDist[277];
    double deadZone1Angle=0.5;
    double deadZone2Angle=2;
    double deadZoneTranslate=0.05;
    double deadZoneToRequiredPos=0.04;
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
    short robotMap2 [ 120 ][ 120 ];


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
    long double robotX;
    long double robotY;
    bool prvyStart=true;
};




























namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    long double robotX;
    long double robotY;
    double robotFi;
    int globalcommand;


    int EncMax=65536;
    double PolohaX;
    double PolohaY;
    double Uhol;
    double GyroUhol;
    double GyroUholOld;
    double deltaUhol;
    int PomEncoderL;
    int PomEncoderR;
    int deltaEncL;
    int deltaEncR;
    double deltaVzdialenostL;
    double deltaVzdialenostR;
    bool prvyStart=true;
    double gyro;

    std::string ipaddress;
    std::vector<RobotCommand> commandQuery;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    //vlakno co cita robot

    bool naviguj;
    double zX;
    double zY;
    double toleranciaUhla;
    int dl;
    int stopall;
    std::thread robotthreadHandle;
 //   pthread_t robotthreadHandle; // handle na vlakno
    int robotthreadID;  // id vlakna
    static void *robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }
    std::thread laserthreadHandle;
  //  pthread_t laserthreadHandle; // handle na vlakno
    int laserthreadID;  // id vlakna
    static void *laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }



    std::thread skeletonthreadHandle;
      //  pthread_t laserthreadHandle; // handle na vlakno
        int skeletonthreadID;  // id vlakna
        static void *skeletonUDPVlakno(void *param)
        {
            std::cout<<"startujem ci co"<<std::endl;
            ((MainWindow*)param)->skeletonprocess();

            return 0;
        }
    QThread Imager;
    void imageViewer();
    void sendRobotCommand(char command,double speed=0,int radius=0);
  //  void autonomousRobotCommand(char command,double speed=0,int radius=0);

    void robotprocess();
    void laserprocess();
    void skeletonprocess();
    void localrobot(TKobukiData &sens);
    void autonomousrobot(TKobukiData &sens);

    int locallaser(LaserMeasurement &laserData);
    int autonomouslaser(LaserMeasurement &laserData);

    void paintThisLidar(LaserMeasurement &laserData);
    LaserMeasurement paintLaserData;
    int updateLaserPicture;
    int updateCameraPicture;
    int updateSkeletonPicture;
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;


    struct sockaddr_in ske_si_me, ske_si_other,ske_si_posli;

    int ske_s,  ske_recv_len;

    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;

#ifdef _WIN32
        int rob_slen;
        int las_slen;
        int ske_slen;
#else
         unsigned int rob_slen;
         unsigned int las_slen;
         unsigned int ske_slen;
#endif
 CKobuki robot;
 TKobukiData sens;
    QTimer *timer;
    std::vector<RobotData> sensorQuerry;
    std::vector<LidarVector> lidarQuerry;
    std::vector<CameraVector> cameraQuerry;
    std::vector< CommandVector> AutonomousCommandQuerry;
    cv::Mat robotPicture;
    cv::Mat AutonomousrobotPicture;

    skeleton kostricka;
private slots:


    void on_pushButton_12_clicked();

    void on_checkBox_clicked(bool checked);

    void on_checkBox_skeleton_clicked(bool checked);

    void on_showMap_clicked(bool checked);

    void on_showCam_clicked(bool checked);

private:
    QImage imgIn;
    bool showCamera;
    bool showLidar;
    bool showSkeleton;
    int datacounter;

    bool applyDelay;
    Ui::MainWindow *ui;
    LaserMeasurement copyOfLaserData;

    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent* event);


    float distPointToLine(float point_x, float point_y, float line_x1, float line_y1, float line_x2, float line_y2);
    double ramp(double speed,double inc,bool *start);
    double euclidDist(double x1,double y1,double x2,double y2);
    bool inRange(double low, double high, double x);
    void go(double speed);
    void turn_around(double angular_velocity);
    void stop();
    void printt();


protected:
    void mousePressEvent(QMouseEvent *event) override;



public slots:
     void setUiValues(QString instruction);
signals:
     void uiValuesChanged(QString newInstruction); ///toto nema telo

};

#endif // MAINWINDOW_H
