#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <boost/thread.hpp>
#include <athomerobot_msgs/sepantaAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "WaveDetect.h"

#include <std_msgs/String.h>

#include "emergency/position.h"
#define R_TIME 1000

using namespace cv;

typedef emergency::position position_msg;


struct  Coordinate
{
    float x, y, z;
};

void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg);
void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
cv::Rect Gesture();
Coordinate ToWorldPose(cv::Rect rectCrop);
void Moveglx(float z);
void Movegly(float x);
void logicCallBack(const std_msgs::String &msg);

ros::Publisher logicPublisher;

WaveDetect waveDetection;
Mat GlobalImage;
Rect accidentPosition;
pcl::PointCloud<pcl::PointXYZ> globalcloud;
bool trueflag = false;

typedef actionlib::SimpleActionClient<athomerobot_msgs::sepantaAction> RClient;
//typedef actionlib::SimpleActionClient<athomerobot_msgs::whatDidYouSayAction> ;
RClient *globalR;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gestureE");
    ros::NodeHandle speechNodeHandle;
    ros::NodeHandle advertiseNodeHandle;
    ros::NodeHandle nodeHandleForGestureLogicSubscribe;
    ros::NodeHandle nodeHandleForKinect;

    image_transport::ImageTransport imageTransport(nodeHandleForKinect);
    image_transport::Subscriber imageSubscriber;
    imageSubscriber = imageTransport.subscribe( "/camera/rgb/image_color", 1, rosImageCallBack);

    ros::NodeHandle nodeHandleForPC;
    ros::Subscriber PCSubscriber = nodeHandleForPC.subscribe("/camera/depth_registered/points", 1, PointCloudCallBack);

    ros::Subscriber taskLogicSubscriber = nodeHandleForGestureLogicSubscribe.subscribe("AUTROBOT_logic_to_Gesture", 10, logicCallBack);
    ROS_INFO("residam1");
    logicPublisher = advertiseNodeHandle.advertise<position_msg>("AUTROBOT_from_Gesture_to_logic", 1);
    ROS_INFO("residam2");
    boost::this_thread::sleep(boost::posix_time::milliseconds(500));

    boost::thread gesture_thread(&Gesture);
    ros::spin();

     gesture_thread.join();
     gesture_thread.interrupt();
    
    trueflag = false;
    return 0;
}
void logicCallBack(const std_msgs::String &msg)
{

    ROS_INFO("logicCallBack: msg: %s", msg.data.c_str());
    if (msg.data == "Start_Find_Gesture" )
    {
        trueflag = true;
    }
    else if (msg.data == "Emergency_Stop" )
    {
      ROS_INFO("stop");
        ros::shutdown();
    }
}


cv::Rect Gesture()
{
    //ros::Rate r(20);
    while (ros::ok)
    {
      //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        //ROS_INFO("staet****************");
        if (trueflag)
        {
          //  ROS_INFO(" a");

            if ((GlobalImage.rows != 0) && (globalcloud.size() != 0))
            {
                //ROS_INFO("b");

                waveDetection.updateImage(GlobalImage);
                waveDetection.updateCloud(globalcloud);
                //ROS_INFO("c");

                if (waveDetection.foundGesture())
                {
                    ROS_INFO("getGesture$$$$$$$$$$$$$$$$$$");
                    accidentPosition = waveDetection.WaveLocation();
                    Coordinate Pose = ToWorldPose(accidentPosition);
                    position_msg msg;
                    msg.command = "GestureFound";
                    msg.x = Pose.x;
                    msg.z = Pose.z;
                    logicPublisher.publish(msg);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                    ros::shutdown();
                }

            }
        }

        //  r.sleep();
    }
    ROS_INFO("tamam");

}

void Movegly(float x)
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
    RGoal.type = "movey";
    RGoal.value = -(x * 100);
    RAction.sendGoal(RGoal);
    //wait for the action to return
    ROS_INFO("Waiting for Movegly to be dONE!");
    bool finished_before_timeout = RAction.waitForResult(ros::Duration(R_TIME));
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
void Moveglx(float z)
{
    ros::Rate loop_rate(20);
    RClient RAction("sepanta_action", true);
    globalR = &RAction;
    ROS_INFO("Waiting for moveglx action server to start.");
    /****** CAUTION!! :
     *
     * will wait for infinite time till server starts, it should be started before!!
     *
     */

    RAction.waitForServer();
    ROS_INFO("moveglx Action server started, sending goal.");
    athomerobot_msgs::sepantaGoal RGoal;


    //nkhCheck to see if every client needs it's own goal
    //int RInput = DEFALT_GOAL_VALUE;
    RGoal.type = "movex";
    RGoal.value = (z * 100);
    RAction.sendGoal(RGoal);
    //wait for the action to return
    ROS_INFO("Waiting for moveglx to be dONE!");
    bool finished_before_timeout = RAction.waitForResult(ros::Duration(R_TIME));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = RAction.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("moveglx finished\n");
        }
        else
        {
            ROS_INFO("Finished moveglx with current State: %s\n",
                     RAction.getState().toString().c_str());
        }

    }
    else
    {
        ROS_INFO("moveglx did not finish before the time out.");

        RAction.cancelGoal();
        RAction.stopTrackingGoal();
    }
}

void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_INFO("image");
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
    imagePointer->image.copyTo(GlobalImage);

}


void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, globalcloud);
}


Coordinate ToWorldPose(cv::Rect rectCrop)  //mode azafe shavad
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud = globalcloud;
    Coordinate World_Pose;
    int cnt_z = 0;
    int cnt_y = 0;
    int cnt_x = 0;
    pcl::PointXYZ p1;
    ROS_INFO("wordl to pose1");
    for (int j = rectCrop.x; j < (rectCrop.x + rectCrop.width); j++)
    {
        for (int i =  rectCrop.y; i < (rectCrop.y + rectCrop.height); i++)
        {
            // w = croppedImage->at(i, j).x;
            // h = croppedImage->at(i, j).y;
            p1.x = cloud->at(j, i).x;
            p1.y = cloud->at(j, i).y;
            p1.z = cloud->at(j, i).z;

            if ( !isnan(p1.x))
            {
                World_Pose.x += p1.x;
                cnt_x++;
            }
            if ( !isnan(p1.y))
            {
                World_Pose.y += p1.y;
                cnt_y++;
            }
            if ( !isnan(p1.z) && (p1.z > 0))
            {
                World_Pose.z += p1.z;
                cnt_z++;
            }
        }
    }
    World_Pose.x = World_Pose.x / cnt_x;
    World_Pose.y = World_Pose.y / cnt_y;
    World_Pose.z = World_Pose.z / cnt_z;

    return World_Pose;
}

