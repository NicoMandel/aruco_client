#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class positionbrc
{
    private:
        ros::NodeHandle nh;
        const std::string world_frame = "map";
        const std::string child_frame = "base_link";
        const std::string pose_topic = "/mavros/local_position/pose";
        tf2_ros::TransformBroadcaster tbr;
        ros::Subscriber pose_subscriber;
        geometry_msgs::TransformStamped tfs;
        void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    public:
        positionbrc(ros::NodeHandle nh);
        ~positionbrc();

};