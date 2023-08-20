#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
class VishgroundAPI {
  public :
    VishgroundAPI(ros::NodeHandle &nh, ros::Rate &rate);

    //API

  private :
    ros::NodeHandle nh_;
    ros::Rate rate_;
  //service and client 
    // ros::ServiceClient command_client;
    // ros::ServiceClient command_tkoff_client;
    // ros::ServiceClient command_land_client;

  //publisher
    
  
  //subscriber
    ros::Subscriber global_position_global_sub;
    ros::Subscriber global_position_rel_alt_sub;

  
  //subscriber callback
    void global_position_global_cb(const sensor_msgs::NavSatFix &msg);
    void global_position_rel_alt_cb(const std_msgs::Float64 &msg);
};