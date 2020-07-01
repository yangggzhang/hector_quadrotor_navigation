#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_quadrotor_actions/base_action.h>
#include <hector_quadrotor_interface/helpers.h>
#include <ros/ros.h>

#include <iterator>
#include <vector>

#include "hector_navigation_msgs/FollowTrajectoryAction.h"
#include "hector_navigation_msgs/Waypoint.h"

namespace hector_quadrotor_actions {

class FollowTrajectoryActionServer {
 public:
  FollowTrajectoryActionServer(ros::NodeHandle nh)
      : follow_trajectory_server_(
            nh, "action/followtrajectory",
            boost::bind(&FollowTrajectoryActionServer::followtrajectoryActionCb,
                        this, _1)) {
    waypoint_pub_ =
        nh.advertise<hector_navigation_msgs::Waypoint>("command/waypoint", 1);

    nh.param<double>("waypoint_dist_tolerance", waypoint_dist_tolerance_, 0.15);
    nh.param<double>("waypoint_yaw_tolerance", waypoint_yaw_tolerance_, 0.55);
    nh.param<double>("final_dist_tolerance", final_dist_tolerance_, 0.05);
    nh.param<double>("final_yaw_tolerance", final_yaw_tolerance_, 0.35);
    nh.param<double>("waypoint_time_in_tolerance", waypoint_time_in_tolerance_,
                     0.2);
    nh.param<double>("final_time_in_tolerance", final_time_in_tolerance_, 1.0);
    nh.param<double>("action_frequency", frequency_, 10);
    nh.param<double>("action_timeout", action_timeout_, 30.0);
  }

  void followtrajectoryActionCb(
      const hector_navigation_msgs::FollowTrajectoryGoalConstPtr &goal) {
    follow_trajectory_server_.enableMotors(true);

    std::vector<hector_navigation_msgs::Waypoint> trajectory = goal->trajectory;

    std::vector<hector_navigation_msgs::Waypoint>::iterator waypoint_it =
        trajectory.begin();

    hector_navigation_msgs::Waypoint waypoint = *waypoint_it;

    ros::Rate r(frequency_);
    ros::Time start = ros::Time::now();
    ros::Time last_time_out_of_tolerance_ = ros::Time::now();

    while (ros::ok() && follow_trajectory_server_.get()->isActive() &&
           waypoint_it != trajectory.end()) {
      if (follow_trajectory_server_.get()->isPreemptRequested()) {
        if (!follow_trajectory_server_.get()->isNewGoalAvailable()) {
          // Stop moving
          hector_navigation_msgs::Waypoint holding_waypoint;
          geometry_msgs::PoseStamped current_pose =
              *follow_trajectory_server_.getPose();
          holding_waypoint.header = current_pose.header;
          holding_waypoint.pose = current_pose.pose;
          holding_waypoint.speed = 0.0;
          waypoint_pub_.publish(holding_waypoint);
        }
        follow_trajectory_server_.get()->setPreempted();
        return;
      }
      // Update to the next

      hector_navigation_msgs::FollowTrajectoryFeedback feedback;
      feedback.current_pose = *follow_trajectory_server_.getPose();
      follow_trajectory_server_.get()->publishFeedback(feedback);

      waypoint = *waypoint_it;
      waypoint.header.frame_id = goal->header.frame_id;
      waypoint.header.stamp = ros::Time::now();
      waypoint_pub_.publish(waypoint);

      if (waypoint_it != trajectory.end()) {
        if (!hector_quadrotor_interface::poseWithinTolerance(
                feedback.current_pose.pose, waypoint.pose,
                waypoint_dist_tolerance_, waypoint_yaw_tolerance_)) {
          last_time_out_of_tolerance_ = ros::Time::now();
        } else if (last_time_out_of_tolerance_ +
                       ros::Duration(waypoint_time_in_tolerance_) <
                   ros::Time::now()) {
          ++waypoint_it;
          continue;
        }
      } else {
        if (!hector_quadrotor_interface::poseWithinTolerance(
                feedback.current_pose.pose, trajectory.back().pose,
                final_dist_tolerance_, final_yaw_tolerance_)) {
          last_time_out_of_tolerance_ = ros::Time::now();
        } else if (last_time_out_of_tolerance_ +
                       ros::Duration(final_time_in_tolerance_) <
                   ros::Time::now()) {
          follow_trajectory_server_.get()->setSucceeded();
          return;
        }
      }

      if (ros::Time::now() > start + ros::Duration(action_timeout_)) {
        follow_trajectory_server_.get()->setAborted();
        return;
      }

      ros::spinOnce();
      r.sleep();
    }
    follow_trajectory_server_.get()->setSucceeded();
    return;
  }

 private:
  hector_quadrotor_actions::BaseActionServer<
      hector_navigation_msgs::FollowTrajectoryAction>
      follow_trajectory_server_;
  ros::Publisher waypoint_pub_;

  double frequency_, waypoint_dist_tolerance_, final_dist_tolerance_,
      waypoint_yaw_tolerance_, final_yaw_tolerance_, action_timeout_,
      waypoint_time_in_tolerance_, final_time_in_tolerance_;
};

}  // namespace hector_quadrotor_actions

int main(int argc, char **argv) {
  ros::init(argc, argv, "follow_trajectory_action");

  ros::NodeHandle nh;
  hector_quadrotor_actions::FollowTrajectoryActionServer server(nh);

  ros::spin();

  return 0;
}
