#include "hector_navigation/hector_navigation_params.h"

namespace hector {
namespace navigation {

HectorNavigationParam::HectorNavigationParam()
    : takeoff_distance_m(KTakeOffDistance_m),
      move_group(KMoveGroup),
      update_rate_hz(KUpdateRate_hz),
      num_slowdown_waypoints(KNumSlowDownPoints),
      planning_attempt(KPlanningAttempt),
      workspace_lowerbound_m(KWorkspaceLowerbound_m),
      workspace_upperbound_m(KWorkspaceUpperbound_m),
      planner_id(KPlannerId) {}

bool HectorNavigationParam::LoadFromRosParams(ros::NodeHandle& ph) {
  if (!ph.getParam("takeoff_distance_m", takeoff_distance_m)) {
    ROS_WARN_STREAM(
        "Using default take off height (m): " << KTakeOffDistance_m);
  }

  if (!ph.getParam("num_slowdown_waypoints", num_slowdown_waypoints)) {
    ROS_WARN_STREAM(
        "Using default number of slow down points: " << KNumSlowDownPoints);
  }

  if (!ph.getParam("move_group", move_group)) {
    ROS_WARN_STREAM("Using default move group : " << KMoveGroup);
  }

  if (!ph.getParam("update_rate_hz", update_rate_hz)) {
    ROS_WARN_STREAM("Using default update rate : " << KUpdateRate_hz << "hz.");
  }

  if (!ph.getParam("planning_attempt", planning_attempt)) {
    ROS_WARN_STREAM("Using default number of planning attempt : "
                    << planning_attempt << ".");
  }

  if (!ph.getParam("planner_id", planner_id)) {
    ROS_WARN_STREAM("Using default planner : " << KPlannerId);
  }

  if (!ph.getParam("workspace_lowerbound_m", workspace_lowerbound_m)) {
    ROS_WARN_STREAM("Using default planning workspace lower bound");
    ROS_WARN_STREAM("x(m) : " << KWorkspaceLowerbound_m[0]
                              << " y(m) : " << KWorkspaceLowerbound_m[1]
                              << " z(m) : " << KWorkspaceLowerbound_m[2]);
  }
  if (!ph.getParam("workspace_upperbound_m", workspace_upperbound_m)) {
    ROS_WARN_STREAM("Using default planning workspace upper bound");
    ROS_WARN_STREAM("x(m) : " << KWorkspaceUpperbound_m[0]
                              << " y(m) : " << KWorkspaceUpperbound_m[1]
                              << " z(m) : " << KWorkspaceUpperbound_m[2]);
  }

  return true;
}

}  // namespace navigation
}  // namespace hector