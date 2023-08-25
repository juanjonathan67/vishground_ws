#include "ros/ros.h"
#include <string>

#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandLong.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"

#include "mavros_msgs/State.h"
#include "sensor_msgs/BatteryState.h"



class VishgroundAPI {
  public :
    VishgroundAPI(ros::NodeHandle &nh, ros::Rate &rate);

    void arm();
    void disarm();

    //API

  private :
    ros::NodeHandle nh_;
    ros::Rate rate_;
  //service and client 
    ros::ServiceClient cmd_cli;
    ros::ServiceClient cmd_arm_cli;
    ros::ServiceClient cmd_tkoff_cli;
    ros::ServiceClient cmd_land_cli;
  //publisher
    
  
  //subscriber
    ros::Subscriber local_position_pose_sub;
    ros::Subscriber local_position_velocity_sub;

    ros::Subscriber global_position_global_sub;
    ros::Subscriber global_position_rel_alt_sub;

    ros::Subscriber global_position_gp_vel_sub;
    ros::Subscriber global_position_compass_hdg_sub;
    ros::Subscriber state_sub;
    ros::Subscriber battery_sub;

  
  //subscriber callback
    void local_position_pose_cb(const geometry_msgs::PoseStamped &msg);
    void local_position_velocity_cb(const geometry_msgs::TwistStamped &msg);

    void global_position_global_cb(const sensor_msgs::NavSatFix &msg);
    void global_position_rel_alt_cb(const std_msgs::Float64 &msg);
    void global_position_gp_vel_cb(const geometry_msgs::TwistStamped &msg);
    void global_position_compass_hdg_cb(const std_msgs::Float64 &msg);
    void state_cb(const mavros_msgs::State &msg);
    void battery_cb(const sensor_msgs::BatteryState &msg);
};