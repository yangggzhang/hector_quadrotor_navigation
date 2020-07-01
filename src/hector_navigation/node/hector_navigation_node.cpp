#include "hector_navigation/hector_navigation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hector_navigation");
  ros::NodeHandle nh, ph("~");
  std::unique_ptr<hector::navigation::HectorQuadrotor> hector_agent =
      hector::navigation::HectorQuadrotor::MakeUniqueFromRosParam(nh, ph);
  if (hector_agent == nullptr) {
    ROS_ERROR("Failed to launch hector quadrotor!");
    return -1;
  }
  ros::AsyncSpinner spinner(1);
  spinner.start();
  hector_agent->Run();
  return 0;
}