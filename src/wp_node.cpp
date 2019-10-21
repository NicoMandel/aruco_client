#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>


// A class which is being called below, receiving a PoseStamped
// from a topic published by the navigator
// and overwrites own attribute posestamped 
// which is published at a frequency large enough not to drop out
class waypointNode
{
    private:
        // stuff for receiving the pose and publishing it
        ros::NodeHandle nh;
        geometry_msgs::PoseStamped pose;
        ros::Publisher pose_publisher;
        ros::Subscriber pose_subscriber;

        // ros::Timer poseTimer;
    public:
        waypointNode(ros::NodeHandle);
        ~waypointNode();

        // methods for receiving the pose and publishing it
        void poseCallback(const geometry_msgs::PoseStampedPtr& msg);
        void publishPose();

        // stuff from the mavros tutorial
        ros::Subscriber state_sub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        mavros_msgs::State current_state;

        void state_callback(const mavros_msgs::StatePtr& msg);
        bool setpoint_ok();
        // void sendPose(const ros::TimerEvent& event);
};

waypointNode::waypointNode(ros::NodeHandle nh)
{
  
    this->pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",5);
    this->pose_subscriber = nh.subscribe("/knowledgebase/nextWP", 1, &waypointNode::poseCallback, this);

    // mavros subscribers - use <mavros_msgs::State>?
    this->state_sub = nh.subscribe("mavros/state", 10, &waypointNode::state_callback, this);
    this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // initial pose - make sure there is one - how to?
    // geometry_msgs::Point translat = geometry_msgs::Point();
    // geometry_msgs::Quaternion q = geometry_msgs::Quaternion();
    this->pose = geometry_msgs::PoseStamped();
    this->pose.header.stamp = ros::Time(0);
    // this->pose.header.frame_id = "map";
    // pose.header.stamp = ros::Time::now();
    // pose.pose.position = translat;
    // pose.pose.orientation = q;
    // this->pose = pose;
    // poseTimer = nh.createTimer(ros::Duration(1.0/20.0),  &waypointNode::sendPose, this);

}

// void waypointNode::sendPose(const ros::TimerEvent& event)
// {
//     this->pose.header.stamp = ros::Time::now();
//     this->pose_publisher.publish(this->pose);
// }

waypointNode::~waypointNode()
{
    // what do i need to do on shutdown?
    // ros::shutdown() ??? 
    // nh::shutdown()?
}

// overwrite the pose of the object, which is being published
void waypointNode::poseCallback(const geometry_msgs::PoseStampedPtr& msg)
{
    this->pose = *msg;
}

// cycle through this method to publish the pose at a given frequency
void waypointNode::publishPose()
{
    this->pose.header.stamp = ros::Time::now();
    this->pose_publisher.publish(this->pose);
}

// callback to overwrite the object's state
void waypointNode::state_callback(const mavros_msgs::StatePtr& msg)
{
    this->current_state = *msg;
}

// This method does not really work. Returns True, if the time is not 0
bool waypointNode::setpoint_ok()
{
    return this->pose.header.stamp != ros::Time(0);
}

// Main part
int main(int argc, char** argv){
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nodeh;
    waypointNode WP_node(nodeh);
    const std::string mode_string = "OFFBOARD";

    // stuff from mavros tutorial - checking if flight controller is connected
    ros::Rate rate(20.0);
    while(ros::ok() && !WP_node.current_state.connected && !WP_node.setpoint_ok()){
        ros::spinOnce();
        rate.sleep();
    }

    // Setting the mode and arming command
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mode_string;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Timer
    ros::Time last_request = ros::Time::now();
    // sending a few setpoints before starting
    // Effectively, this will take 5 seconds @ 20 Hz
    for (int i=100; ros::ok() && i >0; i--){
        WP_node.publishPose();
        ros::spinOnce();
        rate.sleep();
    }

    // This is the actual main loop!
    while(ros::ok){

        if (WP_node.current_state.mode != mode_string && (ros::Time::now()  - last_request > ros::Duration(5.0)))
        { 
            if(WP_node.set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
            ROS_INFO("Offboard Enabled");
            }
        last_request = ros::Time::now();
        } else
        {
            if(!WP_node.current_state.armed && (ros::Time::now()  - last_request > ros::Duration(5.0)))
            {
                if(WP_node.arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle Armed");
                }
            last_request = ros::Time::now();
            }
        }
        
        WP_node.publishPose();
            
                    
        ros::spinOnce();
        rate.sleep();                
    }
    // Do I need to close the object, and if yes, how do I end all subscribers

    // Do I need this shutdown
    // nodeh.shutdown();
    


    return 0;
}