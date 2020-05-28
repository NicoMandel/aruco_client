#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_client/base_link_broadcaster.h>

positionbrc::positionbrc(ros::NodeHandle nh)
{   
    ros::topic::waitForMessage<geometry_msgs::PoseStampedPtr>(pose_topic, nh, ros::Duration(10.0));
    tfs = geometry_msgs::TransformStamped();
    tfs.header.frame_id = world_frame;
    tfs.child_frame_id = child_frame;
    tbr = tf2_ros::TransformBroadcaster();   
    pose_subscriber = nh.subscribe(pose_topic,3, &poseCallback);
}

positionbrc::~positionbrc(){

}

positionbrc::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    try
    {
        tfs.header.stamp = ros::Time::now();
        tfs.transform.translation.x = msg->pose.position.x;
        tfs.transform.translation.y = msg->pose.position.y;
        tfs.transform.translation.z = msg->pose.position.z;
        tfs.transform.rotation.x = msg->pose.orientation.x;
        tfs.transform.rotation.y = msg->pose.orientation.y;
        tfs.transform.rotation.z = msg->pose.orientation.z;
        tfs.transform.rotation.w = msg->pose.orientation.w;

        tbr.sendTransform(tfs);
    }
    catch(ros::Exception& e)
    {
        ROS_INFO(e.what());
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
        ROS_INFO(e.what());
    }

    ros::spin();

    return 0;
}