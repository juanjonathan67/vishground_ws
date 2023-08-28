#include "vishground/vishground_api.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "vishground");
  ros::NodeHandle nh;
  ros::Rate rate(1);

  VishgroundAPI vishgroundapi(nh, rate);

  while(ros::ok()){
    std::cout << "Here" << std::endl;
    vishgroundapi.get_mqtt_client()->subscribe();
    ros::spinOnce();
    rate.sleep();
  }

  // vishgroundapi.arm();

  ros::spin();

  return 0;
}