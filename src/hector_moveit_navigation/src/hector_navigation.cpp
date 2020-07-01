#include "hector_moveit_navigation/hector_navigation.h"

#include <boost/optional.hpp>

namespace hector {
namespace navigation {

HectorQuadrotor::HectorQuadrotor(
    const HectorNavigationParam& params, std::unique_ptr<NavigationAgent> agent,
    std::unique_ptr<NavigationController> trajectory_controller,
    std::unique_ptr<WaypointController> waypoint_controller,
    ros::NodeHandle& nh)
    : params_(params),
      state_(IDLE),
      agent_(std::move(agent)),
      trajectory_controller_(std::move(trajectory_controller)),
      waypoint_controller_(std::move(waypoint_controller)) {
  pose_sub_ = nh.subscribe<nav_msgs::Odometry>(
      "/ground_truth/state", 10, &HectorQuadrotor::update_pose_callback, this);

  enable_motor_service_ =
      nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");

  hector_navigation_server_ =
      nh.advertiseService("hector_waypoint_navigation",
                          &HectorQuadrotor::WaypointNavigationService, this);

  hector_takeoff_server_ = nh.advertiseService(
      "hector_takeoff", &HectorQuadrotor::TakeoffService, this);
}

std::unique_ptr<HectorQuadrotor> HectorQuadrotor::MakeUniqueFromRosParam(
    ros::NodeHandle& nh, ros::NodeHandle& ph) {
  HectorNavigationParam params;
  if (!params.LoadFromRosParams(ph)) {
    ROS_ERROR(
        "Failed to load navigation parameters for hector quadrotor "
        "navigation!");
    return nullptr;
  }

  std::unique_ptr<NavigationAgent> agent =
      std::unique_ptr<NavigationAgent>(new NavigationAgent(params.move_group));

  agent->setPlannerId(params.planner_id);
  agent->setNumPlanningAttempts(params.planning_attempt);
  agent->setWorkspace(
      params.workspace_lowerbound_m[0], params.workspace_lowerbound_m[1],
      params.workspace_lowerbound_m[2], params.workspace_upperbound_m[0],
      params.workspace_upperbound_m[1], params.workspace_upperbound_m[2]);

  std::unique_ptr<NavigationController> navigation_controller =
      std::unique_ptr<NavigationController>(
          new NavigationController("/action/followtrajectory", true));
  if (!navigation_controller->waitForServer(ros::Duration(30))) {
    ROS_ERROR_STREAM(
        "Failed to bring up hector navigation trajectory following "
        "controller!");
    return nullptr;
  } else {
    ROS_INFO_STREAM("Navigation trajectory following controller is up!");
  }

  std::unique_ptr<WaypointController> waypoint_controller =
      std::unique_ptr<WaypointController>(
          new WaypointController("action/waypoint", true));
  if (!waypoint_controller->waitForServer(ros::Duration(30))) {
    ROS_ERROR("Failed to bring up hector waypoint controller!");
    return nullptr;
  } else {
    ROS_INFO_STREAM("Waypoint controller is up!");
  }

  return std::unique_ptr<HectorQuadrotor>(new HectorQuadrotor(
      params, std::move(agent), std::move(navigation_controller),
      std::move(waypoint_controller), nh));
}

bool HectorQuadrotor::EnableMotor() {
  if (state_ != IDLE) {
    ROS_INFO_STREAM("Hector's motor already enabled!");
    return true;
  }
  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;
  enable_motor_service_.call(srv);

  if (srv.response.success) {
    ROS_INFO_STREAM("Motor enabled!");
    state_ = LANDED;
    return true;
  } else {
    ROS_INFO_STREAM("Failed to enable motor!");
    return false;
  }
}

bool HectorQuadrotor::TakeOff(const double& takeoff_distance) {
  if (!current_pose_.is_initialized()) {
    ros::Duration(5.0).sleep();  // sleep for five seconds
    if (!current_pose_.is_initialized()) {
      ROS_ERROR_STREAM("Hector failed to receive pose information!");
      return false;
    }
  }

  if (state_ == IDLE && !EnableMotor()) {
    ROS_ERROR_STREAM("Hector failed to enable motor!");
    return false;
  }

  hector_navigation_msgs::WaypointGoal waypoint_goal;
  waypoint_goal.target_waypoint.header.frame_id = "world";
  waypoint_goal.target_waypoint.pose = current_pose_.get();
  waypoint_goal.target_waypoint.pose.position.z += takeoff_distance;

  waypoint_controller_->sendGoal(waypoint_goal);
  if (!waypoint_controller_->waitForResult()) {
    ROS_WARN("Hector taking off failed at execution!");
    return false;
  } else if (waypoint_controller_->getState() ==
             actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_WARN("Takeoff succeeded");
    state_ = HOVERING;
    return true;
  }
  return false;
}

inline std::vector<double> HectorQuadrotor::PoseToVector(
    const geometry_msgs::Pose& pose) {
  return std::vector<double>{pose.position.x,    pose.position.y,
                             pose.position.z,    pose.orientation.x,
                             pose.orientation.y, pose.orientation.z,
                             pose.orientation.w};
}

inline geometry_msgs::Pose HectorQuadrotor::TransformToPose(
    const geometry_msgs::Transform& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation.x = transform.rotation.x;
  pose.orientation.y = transform.rotation.y;
  pose.orientation.z = transform.rotation.z;
  pose.orientation.w = transform.rotation.w;
  return pose;
}

inline std::vector<hector_navigation_msgs::Waypoint>
HectorQuadrotor::MoveitRobotTrajectoryToWaypointTrajectory(
    const moveit_msgs::RobotTrajectory& msg) {
  std::vector<hector_navigation_msgs::Waypoint> waypoint_trajectory;
  waypoint_trajectory.resize(msg.multi_dof_joint_trajectory.points.size());
  for (int i = 0; i < msg.multi_dof_joint_trajectory.points.size(); ++i) {
    waypoint_trajectory[i].pose =
        TransformToPose(msg.multi_dof_joint_trajectory.points[i].transforms[0]);
    waypoint_trajectory[i].speed = params_.navigation_speed_m_s;
  }
  return waypoint_trajectory;
}

bool HectorQuadrotor::WaypointNavigate(geometry_msgs::Pose& goal,
                                       HectorNavigationErrorCode& error_code) {
  if (!current_pose_.is_initialized()) {
    ROS_ERROR(
        "Failed to get hector's current pose information to plan a "
        "trajectory!");
    error_code = ConnectionFailure;
    return false;
  }

  if (state_ == LANDED && !TakeOff(params_.takeoff_distance_m)) {
    ROS_WARN("Hector failed to takeoff to navigate!");
    return false;
  }

  std::vector<double> start_pose = PoseToVector(current_pose_.get());
  std::vector<double> target_pose = PoseToVector(goal);
  this->agent_->setJointValueTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto return_code = this->agent_->plan(plan);
  if (return_code != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_INFO_STREAM("Hector navigation failed at planning to reach waypoint : "
                    << goal.position.x << goal.position.y << goal.position.z);
    ROS_INFO_STREAM("The returning error code is : " << return_code);
    error_code = PlanFailure;
    return false;
  }

  state_ = NAVIGATING;

  hector_navigation_msgs::FollowTrajectoryGoal trajectory_goal;
  trajectory_goal.header.frame_id = "world";
  trajectory_goal.trajectory =
      MoveitRobotTrajectoryToWaypointTrajectory(plan.trajectory_);
  this->trajectory_controller_->sendGoal(trajectory_goal);
  if (!this->trajectory_controller_->waitForResult()) {
    ROS_ERROR_STREAM("Hector failed to reach goal : "
                     << goal.position.x << goal.position.y << goal.position.z
                     << "during execution!");
    error_code = ExecutionFailure;
    state_ = HOVERING;
    return false;
  }
  state_ = HOVERING;
  error_code = Success;
  return true;
}

bool HectorQuadrotor::Run() {
  ros::Rate rate(params_.update_rate_hz);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return false;
}

void HectorQuadrotor::update_pose_callback(
    const nav_msgs::Odometry::ConstPtr& msg) {
  current_pose_ = boost::make_optional(msg->pose.pose);
}

bool HectorQuadrotor::WaypointNavigationService(
    hector_navigation_msgs::Navigation::Request& req,
    hector_navigation_msgs::Navigation::Response& res) {
  if (state_ == IDLE && !EnableMotor()) {
    ROS_ERROR_STREAM("Failed to enable hector quodrotor's motor!");
    res.return_type = 3;
    return false;
  }
  HectorNavigationErrorCode code;
  if (!WaypointNavigate(req.goal, code)) {
    switch (code) {
      case PlanFailure: {
        res.return_type = 1;
        break;
      }
      case ExecutionFailure: {
        res.return_type = 2;
        break;
      }
      case ConnectionFailure: {
        res.return_type = 4;
        break;
      }
      default: {
        res.return_type = 5;
        break;
      }
    }
  }
  res.return_type = 0;
  return true;
}

bool HectorQuadrotor::TakeoffService(
    hector_navigation_msgs::Takeoff::Request& req,
    hector_navigation_msgs::Takeoff::Response& res) {
  if (state_ == IDLE && !EnableMotor()) {
    res.success = false;
    return false;
  } else if (state_ == LANDED && !TakeOff(req.takeoff_distance_m)) {
    res.success = false;
    return false;
  }
  res.success = true;
  return true;
}
}  // namespace navigation
}  // namespace hector
