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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_msgs/String.h>
#include "athomerobot_msgs/users.h"
#include "emergency/position.h"

typedef athomerobot_msgs::users users_msg;
typedef emergency::position position_msg;

struct  joint
{
    int x, y, z;
};

int state = -1;
void SkeletonLogicCallback(const users_msg &msg);
void logicCallBack(const std_msgs::String &msg);
//void Omnidrive(int x, int y, int w);
ros::Publisher SkeletonPublisher;
ros::Publisher logicPublisher;

joint head;
joint torso;

int id_closer_person =-1;
int userid = -1;
int count = 0;
float img_torsoy;
float img_torsox;
float real_headz;
float real_torsoz;
int main(int argc, char **argv)
{
	 ros::init(argc, argv, "Accident_skeleton");
	 ros::NodeHandle SkeletNodeHandle,advertiseNodeHandle;
	 ros::Subscriber taskLogicSubscriber = SkeletNodeHandle.subscribe("AUTROBOT_logic_to_accident_skeleton", 1, logicCallBack);
     ros::Subscriber SkeletonSubscriber = SkeletNodeHandle.subscribe("AUTROBOTOUT_skeleton", 1, SkeletonLogicCallback);
     logicPublisher = advertiseNodeHandle.advertise<position_msg>("AUTROBOT_from_accident_skeleton_to_logic", 1);
    // omni_drive = n_.advertise<drive_msg>("AUTROBOTIN_omnidrive", 10);

     //state = 1;
     ros::spin();
}
float mean_distance=4;
float MinZ= 10000;
void SkeletonLogicCallback(const users_msg &msg)
{
      // athomerobot_msgs::user item = msg->users.at(i);
        // std::cout<<msg.count<<" count "<<std::endl;

        if ((msg.count != 0)&&((state ==2 )||(state == 1)))
        {
          //  sendToSpeech("stand a front of me and back to me");
            ROS_INFO("seri avval");
            for ( int i = 0 ; i < msg.count ; i++ )
            {
                athomerobot_msgs::user item = msg.users.at(i);
                img_torsox = float(item.Torso.img_x);
                img_torsoy = float(item.Torso.img_y);
                //*******************************real *****************************
                real_torsoz = float(item.Torso.z) / 100;
                real_headz = float(item.Head.z) / 100;

                 std::cout << "img_torsox " << img_torsox << std::endl;
                 std::cout << "torsoy " << img_torsoy << std::endl;
                std::cout << "z " << real_torsoz << std::endl;
                std::cout << "hand_z " << real_headz << std::endl;


                if (img_torsox > 70 && img_torsox < 550 && img_torsoy<400 && img_torsoy>70)
                {
                    if (real_torsoz <mean_distance|| (real_headz <mean_distance))
                    {
                        mean_distance = real_torsoz;
                        id_closer_person = i;
                        head.x = msg.users[i].Head.x;
                        head.y = msg.users[i].Head.y;
                        head.z = msg.users[i].Head.z;
                        torso.x = msg.users[i].Torso.x;
                        torso.y = msg.users[i].Torso.y;
                        torso.z = msg.users[i].Torso.z;           
                        state = 3;
             //            omni_drive(0,0,0);
                         ROS_INFO("stop");
                    }
                }
            }

        }
        if((id_closer_person == -1)&&(state == 1))
        {
            ROS_INFO("shoro be charkhesh");
            //Omnidrive(0,0,30);
            state = 2;
        }
        if ((id_closer_person != -1)&&(state == 3))
        {
            
            
            for (int i= 0; i< msg.count; ++i)
            {
                std::cout<<head.z<<" headavalyez "<<msg.users[userid].Head.z<<" headalan z"<<std::endl;
                std::cout<<torso.y<<" torso x avval "<<msg.users[id_closer_person].Torso.y<<" torso x alan "<<std::endl;  
                std::cout<<head.y<<" headavalye "<<msg.users[userid].Head.y<<"headalan y "<<std::endl;
               // if //age y jadid kamtar az  nesfe tafazole yhead va torso bashe 
                if((((msg.users[i].Torso.y) < (torso.y-abs(torso.y)/3))) || ((msg.users[i].Head.y) < (head.y-abs(head.y-torso.y)/2))) 
                {
                    ROS_INFO("kamtare");
                    ++count;
                    if (count > 5)
                    {
                            ROS_INFO("accident");
                            position_msg msgpos;
                            msgpos.command = "skeletDetect";
                            msgpos.x = (msg.users[id_closer_person].Head.x)-10;//nachasbe be adame
                            msgpos.z = (msg.users[id_closer_person].Head.z )-10;
                            logicPublisher.publish(msgpos);
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));//wait for person to near
                            ros::shutdown();
                    }
                }
            }
        }
}


void logicCallBack(const std_msgs::String &msg)
{

       if (msg.data == "start_emergency" )
    {
    	 ROS_INFO("start_emergency");

        state = 1;
    }
    else if (msg.data == "Emergency_Stop" )
    {
    	state = 3;
    	ros::shutdown();
    }
}
