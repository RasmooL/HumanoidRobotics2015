// Florian

#include "control.hpp"

bool stop_thread = false;

void spinThread() {
  while (!stop_thread) {
    ros::spinOnce();
  }
}
