#include "vishground/vishground_api.hpp"

//Constructor 
VishgroundAPI::VishgroundAPI(ros::NodeHandle &nh, ros::Rate &rate) : nh_(nh), rate_(rate) {
    
    //initialize service and client
    cmd_cli = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    cmd_arm_cli = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    

    //initialize subscriber
    local_position_pose_sub = nh_.subscribe("mavros/local_position/pose", 10, &VishgroundAPI::local_position_pose_cb, this);
    local_position_velocity_sub = nh_.subscribe("mavros/local_position/velocity", 10, &VishgroundAPI::local_position_velocity_cb, this);

    global_position_global_sub = nh_.subscribe("mavros/global_position/global",10,&VishgroundAPI::global_position_global_cb,this);
    global_position_rel_alt_sub = nh_.subscribe("mavros/global_position/rel_alt",10,&VishgroundAPI::global_position_rel_alt_cb,this);
}

//Drone APIs
void VishgroundAPI::arm() {
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  if(cmd_arm_cli.call(arm_cmd)){
    ROS_INFO("Vehicle armed");
  } else {
    ROS_ERROR("Failed to arm vehicle");
  }
}

void VishgroundAPI::disarm() {
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = false;

  if(cmd_arm_cli.call(arm_cmd)){
    ROS_INFO("Vehicle disarmed");
  } else {
    ROS_ERROR("Failed to disarm vehicle");
  }
}

//Callback APIs
void VishgroundAPI::local_position_pose_cb(const geometry_msgs::PoseStamped &msg) {
  ROS_INFO("\nposition x: %lf \nposition y: %lf \nposition z: %lf \n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
}

void VishgroundAPI::local_position_velocity_cb(const geometry_msgs::TwistStamped &msg) {
  ROS_INFO("\nvelocity x: %lf \nvelocity y: %lf \nvelocity z: %lf \n", msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
}

void VishgroundAPI::global_position_global_cb(const sensor_msgs::NavSatFix &msg) {
  ROS_INFO("\nlatitude: %lf \nlongitude: %lf \naltitude: %lf \n", msg.latitude, msg.longitude, msg.altitude);
}

void VishgroundAPI::global_position_rel_alt_cb(const std_msgs::Float64 &msg) {
  ROS_INFO("\nrelative altitude: %lf \n", msg.data);
}