#pragma once

#include <ros/ros.h>

#include <string>

namespace hector {
namespace navigation {

enum HectorState { IDLE, HOVERING, NAVIGATING, LANDED };

enum HectorNavigationErrorCode {
  Null,
  Success,
  PlanFailure,
  ExecutionFailure,
  ConnectionFailure
};

// Default Take Off distance in meter
const double KTakeOffDistance_m = 1.0;

const std::string KMoveGroup = "DroneBody";

const double KUpdateRate_hz = 10;

const int KPlanningAttempt = 10;

const int KNumSlowDownPoints = 3;

// Workspace in x, y and z direction.
const std::vector<double> KWorkspaceLowerbound_m{-20.0, -20.0, 0.05};

const std::vector<double> KWorkspaceUpperbound_m{20.0, 20.0, 5.0};

// Planner option for Moveit!
const std::string KPlannerId = "RRTConnectkConfigDefault";

// Available Planner:
//     - SBLkConfigDefault
//     - ESTkConfigDefault
//     - LBKPIECEkConfigDefault
//     - BKPIECEkConfigDefault
//     - KPIECEkConfigDefault
//     - RRTkConfigDefault
//     - RRTConnectkConfigDefault
//     - RRTstarkConfigDefault
//     - TRRTkConfigDefault
//     - PRMkConfigDefault
//     - PRMstarkConfigDefault
//     - FMTkConfigDefault
//     - BFMTkConfigDefault
//     - PDSTkConfigDefault
//     - STRIDEkConfigDefault
//     - BiTRRTkConfigDefault
//     - LBTRRTkConfigDefault
//     - BiESTkConfigDefault
//     - ProjESTkConfigDefault
//     - LazyPRMkConfigDefault
//     - LazyPRMstarkConfigDefault
//     - SPARSkConfigDefault
//     - SPARStwokConfigDefault

class HectorNavigationParam {
 public:
  HectorNavigationParam();

  bool LoadFromRosParams(ros::NodeHandle &ph);

  double update_rate_hz;

  double takeoff_distance_m;

  int num_slowdown_waypoints;

  std::string move_group;

  int planning_attempt;

  std::string planner_id;

  std::vector<double> workspace_lowerbound_m;

  std::vector<double> workspace_upperbound_m;

};  // namespace scene
}  // namespace navigation
}  // namespace hector
