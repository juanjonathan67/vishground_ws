#include "vishground/vishground_api.hpp"

//Constructor 
VishgroundAPI::VishgroundAPI(ros::NodeHandle &nh, ros::Rate &rate) : nh_(nh), rate_(rate) {
    
    //initialize service and client
    // command_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    

    //initialize subscriber
    global_position_global_sub = nh_.subscribe("mavros/global_position/global",10,&VishgroundAPI::global_position_global_cb,this);

}

void VishgroundAPI::global_position_global_cb(const sensor_msgs::NavSatFix &msg) {
  ROS_INFO("latitude: %lf \nlongitude: %lf \naltitude: %lf \n", msg.latitude, msg.longitude, msg.altitude);
}