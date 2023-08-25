#include "vishground/vishground_api.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "vishground");
  ros::NodeHandle nh;
  ros::Rate rate(20);
  VishgroundAPI vishgroundapi(nh, rate);

  vishgroundapi.arm();

  ros::spin();

  return 0;
}