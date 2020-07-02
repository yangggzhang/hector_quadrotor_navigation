#pragma once

#include <geometry_msgs/Pose.h>
#include <hector_moveit_actions/ExecuteDroneTrajectoryAction.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <vector>

#include "hector_navigation_msgs/FollowTrajectoryAction.h"
#include "hector_navigation_msgs/Navigation.h"
#include "hector_navigation_msgs/Takeoff.h"
#include "hector_navigation_msgs/WaypointAction.h"
#include "hector_navigation_params.h"

namespace hector {
namespace navigation {

using NavigationAgent = moveit::planning_interface::MoveGroupInterface;
using NavigationController = actionlib::SimpleActionClient<
    hector_navigation_msgs::FollowTrajectoryAction>;
using WaypointController =
    actionlib::SimpleActionClient<hector_navigation_msgs::WaypointAction>;

class HectorQuadrotor {
 public:
  HectorQuadrotor() = delete;

  static std::unique_ptr<HectorQuadrotor> MakeUniqueFromRosParam(
      ros::NodeHandle& nh, ros::NodeHandle& ph);

  bool EnableMotor();

  bool TakeOff(const double& takeoff_distance);

  bool Navigate(geometry_msgs::Pose& goal, const double& speed,
                HectorNavigationErrorCode& error_code);

  bool Run();

 private:
  HectorQuadrotor(const HectorNavigationParam& params,
                  std::unique_ptr<NavigationAgent> agent,
                  std::unique_ptr<NavigationController> trajectory_controller,
                  std::unique_ptr<WaypointController> waypoint_controller,
                  ros::NodeHandle& nh);

  HectorNavigationParam params_;

  std::unique_ptr<NavigationAgent> agent_;

  std::unique_ptr<NavigationController> trajectory_controller_;

  std::unique_ptr<WaypointController> waypoint_controller_;

  ros::Subscriber pose_sub_;

  ros::ServiceClient enable_motor_service_;

  ros::ServiceServer hector_navigation_server_;

  ros::ServiceServer hector_takeoff_server_;

  boost::optional<geometry_msgs::Pose> current_pose_;

  void update_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);

  inline std::vector<double> PoseToVector(const geometry_msgs::Pose& pose);

  inline geometry_msgs::Pose TransformToPose(
      const geometry_msgs::Transform& transform);

  inline std::vector<hector_navigation_msgs::Waypoint>
  MoveitRobotTrajectoryToWaypointTrajectory(
      const moveit_msgs::RobotTrajectory& msg, const double& speed);

  HectorState state_;

  bool NavigationService(hector_navigation_msgs::Navigation::Request& req,
                         hector_navigation_msgs::Navigation::Response& res);

  bool TakeoffService(hector_navigation_msgs::Takeoff::Request& req,
                      hector_navigation_msgs::Takeoff::Response& res);
};
}  // namespace navigation
}  // namespace hector