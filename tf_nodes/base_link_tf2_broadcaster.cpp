#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include "aruco_client/base_link_broadcaster.h"
// #include <tf2/transform_datatypes.h>

positionbrc::positionbrc(ros::NodeHandle n)
{   
    this->nh = n;
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>(this->pose_topic, this->nh, ros::Duration(10.0));
    this->tfs = geometry_msgs::TransformStamped();
    // tfs.header.frame_id = world_frame;
    // tfs.child_frame_id = child_frame;
    this->tbr = tf2_ros::TransformBroadcaster();   
    this->pose_subscriber = this->nh.subscribe(this->pose_topic, 3, &positionbrc::poseCallback, this);
}

positionbrc::~positionbrc(){

}

void positionbrc::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    std::cout << "Hello" << std::endl;
    try
    {
	    // tfs = geometry_msgs::TransformStamped();
        tfs.header = msg->header;
        tfs.header.frame_id = this->world_frame;
        tfs.child_frame_id = this->child_frame;
        tfs.transform.translation.x = msg->pose.position.x;
        tfs.transform.translation.y = msg->pose.position.y;
        tfs.transform.translation.z = msg->pose.position.z;
        tfs.transform.rotation.x = msg->pose.orientation.x;
        tfs.transform.rotation.y = msg->pose.orientation.y;
        tfs.transform.rotation.z = msg->pose.orientation.z;
        tfs.transform.rotation.w = msg->pose.orientation.w;

        //tf2::fromMsg(msg, tfs);
        tbr.sendTransform(tfs);
        std::cout << "Goodbye" << std::endl;
    }
    catch(ros::Exception& e)
    {
       ROS_INFO("%s", e.what());
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "base_link_broadcaster");
    ros::NodeHandle n;

    try
    {
        positionbrc pbrc(n);    
    }
    catch(const ros::Exception& e)
    {
        ROS_INFO("%s", e.what());
    }
    ros::spin();

    return 0;
}
