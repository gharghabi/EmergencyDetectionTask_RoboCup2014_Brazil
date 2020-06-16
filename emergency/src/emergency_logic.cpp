#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <athomerobot_msgs/slamactionAction.h>
#include <athomerobot_msgs/grip_fetchAction.h>


#include "athomerobot_msgs/maptools.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "hpdf.h"
#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <pcl/common/time.h>
#include <fstream>
#include <athomerobot_msgs/sepantaAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/package.h>
#include "athomerobot_msgs/arm.h"
#include "athomerobot_msgs/head.h"
#include "emergency/position.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>


#define GF_TIME 180
#define CANCEL_TIME 180
#define TIME_DA 60
#define TIME_DL 30
#define move_TIME 10000
jmp_buf env;
using namespace std;

typedef actionlib::SimpleActionClient<athomerobot_msgs::grip_fetchAction> GFClient;
typedef actionlib::SimpleActionClient<athomerobot_msgs::slamactionAction> SLAMClient;

typedef actionlib::SimpleActionClient<athomerobot_msgs::sepantaAction> RClient;
typedef emergency::position position_msg;

typedef athomerobot_msgs::arm arm_msg_right;
typedef athomerobot_msgs::head head_msg;
typedef std_msgs::Int32 int_msg;

#ifdef HPDF_DLL
void  __stdcall
#else
void
#endif
error_handler  (HPDF_STATUS error_no, HPDF_STATUS   detail_no, void *user_data);
void init();
void goWithSlam(string where);
void logicThread();
void speechLogicCallback(const std_msgs::String &msg);
void GestureLogicCallback(const position_msg &msg);
void motionLogicCallback(const position_msg &msg);
void stopAllAccidentDetection();
bool MakePdf();
void StartAccidentDetection();
void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg);
void print_page  (HPDF_Page page, int page_num);
void print_page_text  (HPDF_Page page, string texts, int i);
bool generatePDF(string path1, string path2, string path3, string path4, string outPutPath);
void draw_image (HPDF_Doc pdf, const char *filename, float x, float y, const char *text);
void FetchAndCarry();
void CheckPositions();
void Movegl(float x, string type);
void skeletonCallback(const position_msg &msg);
ros::Publisher speakPublisher;
ros::Publisher GestureLogicPublisher;
ros::Publisher SkeletonLogicPublisher;
ros::Publisher greenPublisher;
ros::Publisher redPublisher;
ros::Publisher n_head;
ros::Publisher gripper_right;
ros::Publisher arm_right;
ros::Publisher desired_z;


int_msg gripper_msg;
int_msg msg_z;
arm_msg_right my_arm_msg_right;
head_msg my_head_msg;

SLAMClient *globalSLAM;
GFClient *globalGF;

cv::Mat imageGlobal;

RClient *globalR;

ros::ServiceClient client;
athomerobot_msgs::maptools srv;

string placeIDs[] = {"EApartment", "ERoom"};
int state = -1;
string information;
string  ObjectName="shaghayegh";//"coca" //meghdar dehi avalie
string textComp="shaghayegh";
static double first = 0;
static double last = 0;
static double lastTimeFromDetectAccident = 0;
static double firstTimeAccidentDetect = 0;
static double firstSpeech = 0;
static double lastSpeech = 0;
position_msg GestureMessage,SkeletonMessage;
ofstream fs;
string textPath = ros::package::getPath("emergency") + "/src/status.txt";


int main(int argc, char **argv)
{
    ros::init(argc, argv, "emergency");
    ros::NodeHandle speechNodeHandle;
    ros::NodeHandle advertiseNodeHandle;
    ros::NodeHandle LogicNodeHandleToTask;
    ros::NodeHandle n_,greenNodeHandle;
    ros::Subscriber speechSubscriber = speechNodeHandle.subscribe("AUTROBOTIN_speech", 1, speechLogicCallback);
    speakPublisher = advertiseNodeHandle.advertise<std_msgs::String>("/AUTROBOTOUT_speak", 10);/*for example AUTROBOT_from_find_me*/

    ros::Subscriber GestureLogicSubscriber = LogicNodeHandleToTask.subscribe("AUTROBOT_from_Gesture_to_logic", 1, GestureLogicCallback);
    GestureLogicPublisher = advertiseNodeHandle.advertise<std_msgs::String>("AUTROBOT_logic_to_Gesture", 1);

    ros::Subscriber SkeletonLogicSubscriber = LogicNodeHandleToTask.subscribe("AUTROBOT_from_accident_skeleton_to_logic", 1, skeletonCallback);
    SkeletonLogicPublisher = advertiseNodeHandle.advertise<std_msgs::String>("AUTROBOT_logic_to_accident_skeleton", 1);

    greenPublisher = greenNodeHandle.advertise<std_msgs::Bool>("AUTROBOTIN_greenlight", 10);/*for example AUTROBOT_from_find_me*/           
    redPublisher = greenNodeHandle.advertise<std_msgs::Bool>("AUTROBOTIN_redlight", 10);/*for example AUTROBOT_from_find_me*/           
    desired_z = n_.advertise<int_msg>("AUTROBOTIN_desirez", 10);
    gripper_right = n_.advertise<int_msg>("AUTROBOTIN_gripper_right", 10);
    arm_right = n_.advertise<arm_msg_right>("AUTROBOTIN_arm_right", 10);
    n_head = n_.advertise<head_msg>("/AUTROBOTIN_head", 10);

    ros::NodeHandle nodeHandleForKinect;
    image_transport::ImageTransport imageTransport(nodeHandleForKinect);
    image_transport::Subscriber imageSubscriber;
    imageSubscriber = imageTransport.subscribe( "/camera/rgb/image_color", 1, rosImageCallBack);


    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    ros::NodeHandle n_client;
    client = n_client.serviceClient<athomerobot_msgs::maptools>("AUTROBOTINSRV_maptools");//TODO TOPIC CHECK SHAVAD esme service ham bayad dorost she

    fs.open (textPath.c_str());//textPath
    GestureMessage.x = -1000;
    SkeletonMessage.x = -1000;
    state = 0;
    SLAMClient slamAction("slam_action", true);
    globalSLAM = &slamAction ;
    ROS_INFO("wait for slam server");
    slamAction.waitForServer();
    ROS_INFO("connected to slam server");

     boost::thread logic_thread(&logicThread);

    ros::spin();

    logic_thread.join();
    logic_thread.interrupt();
}

void logicThread()
{
    // ros::Rate r(20);
    while (ros::ok)
    {
        if (state == 0) /*init*/
        {
            ROS_INFO("state == 0");
            init();
            state = 1;
        }
        else if (state == 1) /*Enter Apartment*/
        {

            ROS_INFO("state == 1");
            goWithSlam("WDYS");
            state = 2;

        }
        else if (state == 2) /* Enter Room*/
        {
            ROS_INFO("state == 2");
            goWithSlam("WDYS");
            state = 3;
        }
        else if ( state == 3 ) /* start all accident detection */
        {
            ROS_INFO("state == 3");
            first = pcl::getTime();
            StartAccidentDetection();   
            ROS_INFO("boro 4");

            state = 4;
        }
        else if ( state == 4)/*wait for detect accident */
        {
            ROS_INFO("state == 4" );
            last = pcl::getTime();
            if ((last - first) > TIME_DA)
            {
                // std_msgs::String msg;
                // msg.data = "not_found_person";
                // speakPublisher.publish(msg);
                ROS_INFO("timeout DA");
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));//wait for person to near
                state = 5;
            }
        }
        else if (state == 5)  /* Aciident detected request for person location*/
        {

            my_head_msg.tilt = 450; //0 ta 90
            n_head.publish(my_head_msg);
            ROS_INFO("state 5 accident detect request position");
            firstTimeAccidentDetect = pcl::getTime();
            // std_msgs::String msg;
            // msg.data = "request_pose";//start speech
            state = 6;
            ROS_INFO("Roshan kardane cheraghe ghermez");
            std_msgs::Bool b_msg;
            b_msg.data = true;
            redPublisher.publish(b_msg); 
            //speack bege
            firstSpeech = pcl::getTime();
            std_msgs::String msgs;
            msgs.data = "accident_detect";
            speakPublisher.publish(msgs); 
                  //cvSaveImage("test.jpg". ,pSaveImg);

        }
        else if (state == 6)// wait for person location
        {

        ROS_INFO("state 6 wait for position");
         //   ROS_INFO("state == 6");
            lastTimeFromDetectAccident = pcl::getTime();
            if ((lastTimeFromDetectAccident - firstTimeAccidentDetect) > TIME_DL)//position detection time out
            {
                ROS_INFO("timeout DL");
                state = 7;
            }

                  //save image
            string path1 = ros::package::getPath("emergency") + "/png/kinect.png";
            //cv::SaveImage(path1 , imageGlobal); 
            my_head_msg.tilt = 480; //0 ta 90
            n_head.publish(my_head_msg);
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            while (imageGlobal.size().width==0)
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));

            }

            cv::imshow("salam",imageGlobal);
            cv::waitKey(5);
            cv::Size  size(500,500);
            cv::resize(imageGlobal,imageGlobal,size);
            cv::imwrite( path1, imageGlobal );

            CheckPositions();//check posion if possible move to person

        }
        else if (state == 7)
        {
            stopAllAccidentDetection();//time out all accident detection
            ROS_INFO("state == 7");

            ROS_INFO("Khamysh kardane cheragh");
            std_msgs::Bool b_msg;
            b_msg.data = false;
            redPublisher.publish(b_msg);

            std_msgs::String msg;
            msg.data = "start_question";
            speakPublisher.publish(msg);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            state = -2;//wait for speech
        }
        else if (state == 8)
        {

            ROS_INFO("state == 8");
            srv.request.command = "savemarkedpoint";
            srv.request.id = "AccidentPosition";
            client.call(srv);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            srv.request.command = "savemap";
            srv.request.id = "";
            client.call(srv);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            state = 9;//har vaght slam naghsharo ferestad boro state badi
            MakePdf();//TODO motmen nistam kare slam tamum shode bashe ke man pdf besazam inja az slam ye chizi begir
        }
        else if (state == 9)
        {
            ROS_INFO("state == 9");
            // cout<<"get object name "<<ObjectName<<endl;
            // ObjectName = "shaghayegh";//??????????yadet bashe bardari baraye in ke grip run nashe gozashtim
            // if (ObjectName == "shaghayegh");
            // {
            //     FetchAndCarry();
            // }
            state  = 10;
        }
        else if (state == 10)  //waiting for ambulanc and guid him tu patient
        {
            ROS_INFO("state == 10");
            goWithSlam("EA");
            state = 11;
        }
        else if ( state == 11)//wait for ambulance
        {
            std_msgs::String msg;
            msg.data = "beep";//TODO roshan kardane cheragh va speech modam bege komakam kon
            speakPublisher.publish(msg);
            int k= 0;
            while(k<10)
            {
                std_msgs::Bool b_msg;
                b_msg.data = true;
                redPublisher.publish(b_msg); 

                boost::this_thread::sleep(boost::posix_time::milliseconds(500));//wait for person to near
                
                b_msg.data = false;
                redPublisher.publish(b_msg); 
                 boost::this_thread::sleep(boost::posix_time::milliseconds(500));//wait for person to near
                 ++k;
            }


            state = 12;
    
        }
        else if(state == 12)//back to patient
        {
            goWithSlam("$AccidentPosition");
            state = 13;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));//wait for person to near
    }
}

void goWithSlam(string where)
{

    ROS_INFO("Going %s with slam...", where.c_str());
    athomerobot_msgs::slamactionGoal interfacegoal;
    interfacegoal.x = 0;
    interfacegoal.y = 0;
    interfacegoal.yaw = 0;
    interfacegoal.ID = where;

    //globalSLAM->sendGoal(interfacegoal);
    ROS_INFO("goal sent to slam... waiting for reach there.");
    cout << where << endl;
   
   
    ros::Rate r(20);
    while (ros::ok())
    {


            actionlib::SimpleClientGoalState state = globalSLAM->sendGoalAndWait(interfacegoal, ros::Duration(100000), ros::Duration(10000));
            //bool finished_before_timeout = globalSLAM->waitForResult(ros::Duration(SLAM_TIME));
            ROS_INFO("SALAM.... !");
            //if (finished_before_timeout)
            if (state == actionlib::SimpleClientGoalState::PREEMPTED)
            {
                //actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("SLAM PREEMPTED");
            }
             if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                //actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("SLAM SUCCEEDED");
                break;
            }

             if (state == actionlib::SimpleClientGoalState::ABORTED)
            {
                //actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("SLAM ABORTED");
            }

        
        r.sleep();
    }
   
}

//     void goWithSlam(string where)
// {
   
//     //edwin's code :D
//     int R_TIME = 10000;
//     ROS_INFO("Going %s with slam...", where.c_str());
//     athomerobot_msgs::slamactionGoal interfacegoal;
//     interfacegoal.x = 0;
//     interfacegoal.y = 0;
//     interfacegoal.yaw = 0;
//     interfacegoal.ID = where;

//     //globalSLAM->sendGoal(interfacegoal);
//     ROS_INFO("goal sent to slam... waiting for reach there.");

//     globalSLAM->sendGoal(interfacegoal);
//     bool finished_before_timeout = globalSLAM->waitForResult(ros::Duration(R_TIME));
//     actionlib::SimpleClientGoalState state = globalSLAM->getState();
//     if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
//     {
//         ROS_INFO("Slam finished\n");
//     }
//     else
//     {
//         ROS_INFO("Finished Slam with current State: %s\n",
//                  globalSLAM->getState().toString().c_str());
//     }

// }



void speechLogicCallback(const std_msgs::String &msg)
{
    //std::cout << msg.data << std::endl;
     int totaltime=60;
    std::size_t againStr = (msg.data).find(textComp);
    std::size_t found = (msg.data).find("sepanta:");
    std::size_t foundObject = (msg.data).find("ObjectName:");
    if ((state == -2)&&(firstSpeech!=0))
    {
        lastSpeech = pcl::getTime();
        if((lastSpeech - firstSpeech)>totaltime)
        {
                fs.close ();
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                // ROS_INFO("speech question fineshed");
                state = 8;               
        }
    }
    if (msg.data == "detect_accident" )
    {
        if (state == 4)
        {
            std_msgs::String msg;
            msg.data = "accident occured";//TODO roshan kardane cheragh
            speakPublisher.publish(msg);
            state = 5;
        }
    }
    else if ((found != std::string::npos) && !(againStr!=std::string::npos))
    {
        firstSpeech = pcl::getTime();   
        information = msg.data;
        information.erase(0,8);
        std::cout<<information<<" information "<<std::endl;
        fs << information << "\n";
        textComp = msg.data;
    }
    else if (msg.data=="end_question")
    {
        fs.close ();
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        // ROS_INFO("speech question fineshed");
        state = 8;   
    }
    else if (foundObject != std::string::npos)
    {
        string a;
        a = msg.data;
        a.erase(0,11);
        // boost::replace_last(a, "ObjectName:","");
        ObjectName = a;
        string informationObject = "It is the object he wanted: ";
        std::cout<<information<<" information "<<std::endl;
        fs << informationObject;
        fs << a << "\n";
        std::cout<<a<<"   *object name "<<std::endl;
    }
    else if(msg.data == "ready")
    {
        std_msgs::Bool b_msg;
        b_msg.data = true;
        greenPublisher.publish(b_msg);
    }
    else if(msg.data == "stop")
    {
        std_msgs::Bool b_msg;
        b_msg.data = false;
        greenPublisher.publish(b_msg);
    }
}

void GestureLogicCallback(const position_msg &msg)
{
    ROS_INFO("findMeLogicCallback: msg: %s", msg.command.c_str());
    GestureMessage = msg;
    if (msg.command == "GestureFound")
    {
        if ( state == 4)
        {
            state = 5;//TODO age hamzaman tashikhis bedan fajea mishe
        }
    }
}

void init()
{

    my_head_msg.tilt = 512; //0 ta 90
    n_head.publish(my_head_msg);

    arm_msg_right my_arm_msg;
    my_arm_msg.shoulder_pitch = 3000;
    my_arm_msg.shoulder_roll = 2048;
    my_arm_msg.elbow = 2048;
    my_arm_msg.wrist_pitch = 2048;
    my_arm_msg.wrist_roll = 2048;

    arm_right.publish(my_arm_msg);  
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    msg_z.data = 390;//beyne 2000 ta 3000 vali 2000 balast
    desired_z.publish(msg_z);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    gripper_msg.data = 0;
    gripper_right.publish(gripper_msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(500));

}


bool MakePdf()
{
    string path1 = ros::package::getPath("emergency") + "/png/map1.png";
    string path2 = ros::package::getPath("emergency") + "/png/map2.png";
    string path3 = ros::package::getPath("emergency") + "/png/kinect.png";;
    string path4 = ros::package::getPath("emergency") + "/src/status.txt";
    string outPutPath = ros::package::getPath("emergency") + "/pdf/AUTAthome_patient";
    bool back = generatePDF(path1, path2, path3, path4, outPutPath);

    //cv::waitKey(50000000);
    sleep(2);
    //Send To Flash Memory
    if (back)
    {
        string  mountPath = ros::package::getPath("emergency") + "/bash/mount1.sh";
        system(mountPath.c_str());

        mountPath = ros::package::getPath("emergency") + "/bash/mount2.sh";
        system(mountPath.c_str());

        mountPath = ros::package::getPath("emergency") + "/bash/mount3.sh";;
        system(mountPath.c_str());
    }
    else
    {
        cout << "File NOT Generated!!" << endl;
        return false;
    }
    return true;
}



#ifdef HPDF_DLL
void  __stdcall
#else
void
#endif
error_handler  (HPDF_STATUS   error_no, HPDF_STATUS   detail_no, void *user_data)
{
    printf ("ERROR: error_no=%04X, detail_no=%u\n", (HPDF_UINT)error_no,
            (HPDF_UINT)detail_no);
    longjmp(env, 1);
}


void print_page  (HPDF_Page   page,  int page_num)
{
    char buf[50];

    HPDF_Page_SetWidth (page, 800);
    HPDF_Page_SetHeight (page, 800);

    HPDF_Page_BeginText (page);
    HPDF_Page_MoveTextPos (page, 50, 740);
    snprintf(buf, 50, "Page:%d", page_num);

    HPDF_Page_ShowText (page, buf);
    HPDF_Page_EndText (page);
}
void print_page_text  (HPDF_Page   page,  string texts, int i)
{
    char buf[50];

    HPDF_Page_SetWidth (page, 800);
    HPDF_Page_SetHeight (page, 800);

    HPDF_Page_BeginText (page);
    HPDF_Page_MoveTextPos (page, 50, 740 - i);

    snprintf(buf, 50, texts.c_str());
    HPDF_Page_ShowText (page, buf);
    HPDF_Page_EndText (page);
}

void
draw_image (HPDF_Doc     pdf,
            const char  *filename,
            float        x,
            float        y,
            const char  *text)
{
    const char *FILE_SEPARATOR = "/";

    char filename1[255];

    HPDF_Page page = HPDF_GetCurrentPage (pdf);
    HPDF_Image image;

    strcpy(filename1, "");
    strcat(filename1, FILE_SEPARATOR);
    strcat(filename1, filename);

    image = HPDF_LoadPngImageFromFile (pdf, filename1);

    /* Draw image to the canvas. */
    HPDF_Page_DrawImage (page, image, x, y, HPDF_Image_GetWidth (image),
                         HPDF_Image_GetHeight (image));

}

bool generatePDF(string path1, string path2, string path3, string path4, string outPutPath)
{
    HPDF_Doc  pdf;
    HPDF_Font font;
    HPDF_Page page[4];
    HPDF_Outline root;
    HPDF_Outline outline[4];
    HPDF_Destination dst;
    char fname[256];
    ROS_INFO("2***************");
    strcpy (fname, outPutPath.c_str());
    ROS_INFO("3******************");
    strcat (fname, ".pdf");
    ROS_INFO("4******************");
    pdf = HPDF_New (error_handler, NULL);
    ROS_INFO("5******************");
    if (!pdf)
    {
        ROS_INFO("1");
        printf ("error: cannot create PdfDoc object\n");
        return 1;
    }
    if (setjmp(env))
    {
        HPDF_Free (pdf);
        return 1;
    }

    /* create default-font */
    font = HPDF_GetFont (pdf, "Helvetica", NULL);

    /* Set page mode to use outlines. */
    HPDF_SetPageMode(pdf, HPDF_PAGE_MODE_USE_OUTLINE);

    /* Add 3 pages to the document. */
    page[0] = HPDF_AddPage (pdf);
    HPDF_Page_SetFontAndSize (page[0], font, 30);
    //print_page(page[0], 1);

    draw_image (pdf, path1.c_str(), 50, HPDF_Page_GetHeight (page[0]) - 620, "");

    page[1] = HPDF_AddPage (pdf);
    HPDF_Page_SetFontAndSize (page[1], font, 30);
    //print_page(page[1], 2);

    draw_image (pdf, path2.c_str(), 50, HPDF_Page_GetHeight (page[1]) - 620, "");

    page[2] = HPDF_AddPage (pdf);
    HPDF_Page_SetFontAndSize (page[2], font, 30);
    //print_page(page[2], 3);
    draw_image (pdf, path3.c_str(), 50, HPDF_Page_GetHeight (page[2]) - 620, "");

    page[3] = HPDF_AddPage (pdf);
    HPDF_Page_SetFontAndSize (page[3], font, 30);
    //print_page(page[3], 4);
    ifstream file(path4.c_str());
    string str;
    int i = 50;
    while (getline(file, str))
    {

        print_page_text(page[3], str, i);
        i += 50;
    }



    /* create outline root. */
    root = HPDF_CreateOutline (pdf, NULL, "OutlineRoot", NULL);
    HPDF_Outline_SetOpened (root, HPDF_TRUE);

    outline[0] = HPDF_CreateOutline (pdf, root, "page1", NULL);
    outline[1] = HPDF_CreateOutline (pdf, root, "page2", NULL);

    /* create outline with test which is ISO8859-2 encoding */
    outline[2] = HPDF_CreateOutline (pdf, root, "ISO8859-2 text ÓÔÕÖ×ØÙ",
                                     HPDF_GetEncoder (pdf, "ISO8859-2"));

    /* create destination objects on each pages
     * and link it to outline items.
     */
    dst = HPDF_Page_CreateDestination (page[0]);
    HPDF_Destination_SetXYZ(dst, 0, HPDF_Page_GetHeight(page[0]), 1);
    HPDF_Outline_SetDestination(outline[0], dst);
    //  HPDF_Catalog_SetOpenAction(dst);

    dst = HPDF_Page_CreateDestination (page[1]);
    HPDF_Destination_SetXYZ(dst, 0, HPDF_Page_GetHeight(page[1]), 1);
    HPDF_Outline_SetDestination(outline[1], dst);

    dst = HPDF_Page_CreateDestination (page[2]);
    HPDF_Destination_SetXYZ(dst, 0, HPDF_Page_GetHeight(page[2]), 1);
    HPDF_Outline_SetDestination(outline[2], dst);

    dst = HPDF_Page_CreateDestination (page[3]);
    HPDF_Destination_SetXYZ(dst, 0, HPDF_Page_GetHeight(page[3]), 1);
    HPDF_Outline_SetDestination(outline[3], dst);
    /* save the document to a file */
    HPDF_SaveToFile (pdf, fname);

    /* clean up */
    HPDF_Free (pdf);
    return true;
}
void FetchAndCarry( )
{
    ros::Rate loop_rate(20);
    /******** nkhStart creating actions ********/
    GFClient GFAction("Grip_Emergency_action", true);
    globalGF = &GFAction;

    ROS_INFO("Waiting for GripAndFetch action server to start.");
    /****** CAUTION!! :
     *
     * will wait for infinite time till server starts, it should be started before!!
     *
     */
    GFAction.waitForServer();
    ROS_INFO("pickAndPlace Action server started, sending goal.");
    athomerobot_msgs::grip_fetchGoal GFGoal;

    //nkhCheck to see if every client needs it's own goal
    // Check DONE with Esmaeil
    //GFGoal.input = ObjectName;

    GFGoal.destination = "AccidentPosition";
    GFGoal.object = ObjectName;
    GFAction.sendGoal(GFGoal);
    //ppAction.sendGoal(ppGoal);
    //wait for the action to return
    ROS_INFO("Waiting for Pick and Place to be dONE!");
    //bool finished_before_timeout = ppAction.waitForResult(ros::Duration(PP_TIME));
    //if (finished_before_timeout)
    actionlib::SimpleClientGoalState state = GFAction.sendGoalAndWait(GFGoal, ros::Duration(GF_TIME), ros::Duration(CANCEL_TIME));
    if (state != actionlib::SimpleClientGoalState::PREEMPTED)
    {
        //actionlib::SimpleClientGoalState state = ppAction.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Fetch and Carry Action finished\n");
        }
        else
        {
            ROS_INFO("Finished Fetch and Carry with current State: %s\n",
                    GFAction.getState().toString().c_str());
        }

    }
    else
    {
        ROS_INFO("Fetch and Carry Action did not finish before the time out.");
        GFAction.stopTrackingGoal();
        //ppAction.cancelGoal();

    }
    //nkhCheck this may not needed

}
void stopAllAccidentDetection()
{
    // position_msg msg;
    // msg.command = "Emergency_Stop";
    // GestureLogicPublisher.publish(msg);
    
    // msg.command = "Emergency_Stop";
    // motionLogicPublisher.publish(msg);

    std_msgs::String msg;
    msg.data = "Emergency_Stop";
    GestureLogicPublisher.publish(msg);

    msg.data = "Emergency_Stop";
    SkeletonLogicPublisher.publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(500));//wait for person to near

}

void skeletonCallback(const position_msg &msg)
{
    SkeletonMessage = msg;
    //std::cout<<SkeletonMessage.x <<" "<<SkeletonMessage.z<<" ";
    SkeletonMessage.x/= 100;
    SkeletonMessage.z/= 100;

    if (msg.command == "skeletDetect")
    {
        if (state == 4)
            state = 5;
    }
}
void Movegl(float x, string type)
{
    ros::Rate loop_rate(20);
    RClient RAction("sepanta_action", true);//******************
    globalR = &RAction;
    ROS_INFO("Waiting for Movegly server to start.");
    /****** CAUTION!! :
     *
     * will wait for infinite time till server starts, it should be started before!!
     *
     */

    RAction.waitForServer();
    ROS_INFO("Movegly Action server started, sending goal.");
    athomerobot_msgs::sepantaGoal RGoal;


    //nkhCheck to see if every client needs it's own goal
    //int RInput = DEFALT_GOAL_VALUE;
    if (type == "movey")
    {
        std::cout<<x<<" y "<<std::endl;
        RGoal.type = "movey";
        RGoal.value = (x * 100);//x
    }
    else
    {
        
        std::cout<<x<<" x "<<std::endl;

        RGoal.type = "movex";//z
        RGoal.value = (x * 100);
    }

    RAction.sendGoal(RGoal);
    //wait for the action to return
    ROS_INFO("Waiting for Movegly to be dONE!");
    bool finished_before_timeout = RAction.waitForResult(ros::Duration(move_TIME));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = RAction.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Movegly Action finished\n");
        }
        else
        {
            ROS_INFO("Finished Movegly with current State: %s\n",
                     RAction.getState().toString().c_str());
        }
    }
    else
    {
        ROS_INFO("Movegly did not finish before the time out.");

        RAction.cancelGoal();
        RAction.stopTrackingGoal();
    }
}
void StartAccidentDetection()
{

    std_msgs::String msg;
    msg.data = "Start_Find_Gesture";//start gesture
    GestureLogicPublisher.publish(msg);

   // msg.data = "start_emergency";//start speech
    //speakPublisher.publish(msg);
    position_msg command;

    msg.data = "start_emergency";//start speech
    SkeletonLogicPublisher.publish(msg);


    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));//wait for person to near
}
void CheckPositions()//based on task priorities
{
   // ROS_INFO("checkPosition");
    if(SkeletonMessage.x!= -1000 )
    {
        cout<<SkeletonMessage.x<< " Person X"<<endl;
        cout<<SkeletonMessage.z<<" Person z" <<endl;
        if (SkeletonMessage.z -0.7>0)
            Movegl(SkeletonMessage.z -0.7 , "movex");
        Movegl(SkeletonMessage.x , "movey");
        state = 7;
    }
    else if (GestureMessage.x != -1000)
    {
        Movegl(GestureMessage.x , "movey");
        Movegl(GestureMessage.z , "movex");
        state = 7;
    }
}
int counterimage = 0;
void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg)
{

    cv_bridge::CvImagePtr imagePointer;
    try
    {
        imagePointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    ++counterimage;
    if (counterimage > 7)
    {
        imageGlobal = imagePointer->image;
        // imshow("windowerror", imageGlobal);
        // waitKey(3);  
    }
    //DetectColor("yellow");
    //DetectFaceForBlob();
}