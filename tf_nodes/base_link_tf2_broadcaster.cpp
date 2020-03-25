#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>


// map is the base for tf for px4, as seen in mavros/launch/px4_config.yaml
const std::string world_frame = "map";
const std::string child_frame = "base_link";

void poseCallback(const geometry_msgs::PoseStampedPtr& msg){
    static tf2_ros::TransformBroadcaster tbr;
    geometry_msgs::TransformStamped br;

    br.header.stamp = ros::Time::now();
    br.header.frame_id = world_frame;
    br.child_frame_id = child_frame;
    br.transform.translation.x = msg->pose.position.x;
    br.transform.translation.y = msg->pose.position.y;
    br.transform.translation.z = msg->pose.position.z;
    tf2::Quaternion q;
    br.transform.rotation.x = msg->pose.orientation.x;
    br.transform.rotation.y = msg->pose.orientation.y;
    br.transform.rotation.z = msg->pose.orientation.z;
    br.transform.rotation.w = msg->pose.orientation.w;

    tbr.sendTransform(br);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "base_link_broadcaster");
    ros::NodeHandle n;
    std::string topic = "/mavros/local_position/pose";
    ros::topic::waitForMessage<geometry_msgs::PoseStampedPtr>(topic, n, ros::Duration(10.0));

    ros::Subscriber sub = n.subscribe(topic, 3, &poseCallback);
    ros::spin();
    return 0;
}