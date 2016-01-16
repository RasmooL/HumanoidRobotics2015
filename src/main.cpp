// Group C

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>

#include "optim.hpp"
#include "control.hpp"
#include "load.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "imitation");

  stop_thread = false;

  ros::NodeHandle n;
  ros::Rate rate_sleep(50);
  tf::TransformListener *listener = new tf::TransformListener();
  Nao_control control(listener);
  while (ros::ok()) {}

  return 0;
}
