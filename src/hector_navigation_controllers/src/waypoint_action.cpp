#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_navigation_msgs/Waypoint.h>
#include <hector_quadrotor_actions/base_action.h>
#include <hector_quadrotor_interface/helpers.h>
#include <ros/ros.h>

#include "hector_navigation_msgs/WaypointAction.h"

namespace hector_quadrotor_actions {

class WaypointActionServer {
 public:
  WaypointActionServer(ros::NodeHandle nh)
      : waypoint_server_(
            nh, "action/waypoint",
            boost::bind(&WaypointActionServer::waypointActionCb, this, _1)) {
    waypoint_pub_ =
        nh.advertise<hector_navigation_msgs::Waypoint>("command/waypoint", 1);

    nh.param<double>("dist_tolerance", dist_tolerance_, 0.05);
    nh.param<double>("yaw_tolerance", yaw_tolerance_, 0.35);
    nh.param<double>("time_in_tolerance", time_in_tolerance_, 1.0);
    nh.param<double>("action_frequency", frequency_, 10);
    nh.param<double>("action_timeout", action_timeout_, 30.0);
  }

  void waypointActionCb(
      const hector_navigation_msgs::WaypointGoalConstPtr &goal) {
    waypoint_server_.enableMotors(true);

    hector_navigation_msgs::Waypoint waypoint = goal->target_waypoint;

    ros::Rate r(frequency_);
    ros::Time start = ros::Time::now();
    ros::Time last_time_out_of_tolerance_ = ros::Time::now();

    while (ros::ok() && waypoint_server_.get()->isActive()) {
      if (waypoint_server_.get()->isPreemptRequested()) {
        if (!waypoint_server_.get()->isNewGoalAvailable()) {
          // Stop moving
          hector_navigation_msgs::Waypoint holding_waypoint;
          geometry_msgs::PoseStamped current_pose = *waypoint_server_.getPose();
          holding_waypoint.header = current_pose.header;
          holding_waypoint.pose = current_pose.pose;
          holding_waypoint.speed = 0.0;
          waypoint_pub_.publish(holding_waypoint);
        }
        waypoint_server_.get()->setPreempted();
        return;
      }

      waypoint.header.stamp = ros::Time::now();
      waypoint_pub_.publish(waypoint);

      hector_navigation_msgs::WaypointFeedback feedback;
      feedback.current_pose = *waypoint_server_.getPose();
      waypoint_server_.get()->publishFeedback(feedback);

      if (!hector_quadrotor_interface::poseWithinTolerance(
              feedback.current_pose.pose, goal->target_waypoint.pose,
              dist_tolerance_, yaw_tolerance_)) {
        last_time_out_of_tolerance_ = ros::Time::now();
      } else if (last_time_out_of_tolerance_ +
                     ros::Duration(time_in_tolerance_) <
                 ros::Time::now()) {
        waypoint_server_.get()->setSucceeded();
        return;
      }

      if (ros::Time::now() > start + ros::Duration(action_timeout_)) {
        waypoint_server_.get()->setAborted();
        return;
      }

      ros::spinOnce();
      r.sleep();
    }
  }

 private:
  hector_quadrotor_actions::BaseActionServer<
      hector_navigation_msgs::WaypointAction>
      waypoint_server_;
  ros::Publisher waypoint_pub_;

  double frequency_, dist_tolerance_, yaw_tolerance_, action_timeout_,
      time_in_tolerance_;
};

}  // namespace hector_quadrotor_actions

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoint_action");

  ros::NodeHandle nh;
  hector_quadrotor_actions::WaypointActionServer server(nh);

  ros::spin();

  return 0;
}
