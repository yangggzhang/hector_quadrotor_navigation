#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <hector_navigation_msgs/Waypoint.h>
#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_interface/limiters.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>  // for tf::getPrefixParam()
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <boost/thread/mutex.hpp>
#include <cmath>
#include <cstdlib>
#include <limits>

#include "hector_navigation_controllers/waypoint_handle.h"

namespace hector_navigation_controllers {

using namespace hector_quadrotor_interface;

class WaypointController
    : public controller_interface::Controller<QuadrotorInterface> {
 public:
  WaypointController()
      : pose_command_valid_(false), twist_limit_valid_(false) {}

  virtual ~WaypointController() {
    pose_subscriber_.shutdown();
    twist_limit_subscriber_.shutdown();
  }

  virtual bool init(QuadrotorInterface *interface, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    // get interface handles
    pose_ = interface->getPose();
    twist_ = interface->getTwist();
    motor_status_ = interface->getMotorStatus();

    root_nh.param<std::string>("base_link_frame", base_link_frame_,
                               "base_link");
    root_nh.param<std::string>("world_frame", world_frame_, "/world");
    root_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_,
                               "base_stabilized");

    // resolve frames
    tf_prefix_ = tf::getPrefixParam(root_nh);
    world_frame_ = tf::resolve(tf_prefix_, world_frame_);
    base_link_frame_ = tf::resolve(tf_prefix_, base_link_frame_);
    base_stabilized_frame_ = tf::resolve(tf_prefix_, base_stabilized_frame_);

    // Initialize PID controllers
    pid_.x.init(ros::NodeHandle(controller_nh, "x"));
    pid_.y.init(ros::NodeHandle(controller_nh, "y"));
    pid_.z.init(ros::NodeHandle(controller_nh, "z"));
    pid_.yaw.init(ros::NodeHandle(controller_nh, "yaw"));
    pid_.speed.init(ros::NodeHandle(controller_nh, "speed"));

    position_limiter_.init(controller_nh);

    // Setup pose visualization marker output
    initMarker(root_nh.getNamespace());
    marker_publisher_ =
        root_nh.advertise<visualization_msgs::Marker>("command/pose_marker", 1);

    // Initialize inputs/outputs
    waypoint_input_ = interface->addInput<WaypointCommandHandle>("waypoint");
    twist_input_ = interface->addInput<TwistCommandHandle>("pose/twist");
    twist_limit_input_ =
        interface->addInput<TwistCommandHandle>("pose/twist_limit");
    twist_output_ = interface->addOutput<TwistCommandHandle>("twist");

    // subscribe to commanded pose and velocity
    pose_subscriber_ = root_nh.subscribe<hector_navigation_msgs::Waypoint>(
        "command/waypoint", 1,
        boost::bind(&WaypointController::waypointCommandCallback, this, _1));
    twist_limit_subscriber_ = root_nh.subscribe<geometry_msgs::Twist>(
        "command/twist_limit", 1,
        boost::bind(&WaypointController::twistLimitCallback, this, _1));

    return true;
  }

  void reset() {
    pid_.x.reset();
    pid_.y.reset();
    pid_.z.reset();
    pid_.yaw.reset();
    pid_.speed.reset();

    // Set commanded pose to robot's current pose
    updateWaypointCommand(pose_->pose(), 0.0);
    pose_command_valid_ = false;
  }

  virtual void starting(const ros::Time &time) { reset(); }

  virtual void stopping(const ros::Time &time) {
    twist_output_->stop();
    pose_command_valid_ = false;
    //    twist_limit_valid_ = false;
  }

  inline double GetSpeed(const geometry_msgs::Vector3 &vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
  }

  void waypointCommandCallback(
      const hector_navigation_msgs::WaypointConstPtr &command) {
    boost::mutex::scoped_lock lock(command_mutex_);

    ros::Time start_time = command->header.stamp;
    if (start_time.isZero()) start_time = ros::Time::now();
    if (!isRunning()) this->startRequest(start_time);

    updateWaypointCommand(*command);
  }

  void twistLimitCallback(const geometry_msgs::TwistConstPtr &limit) {
    boost::mutex::scoped_lock lock(command_mutex_);

    twist_limit_ = *limit;
    twist_limit_valid_ = true;
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    boost::mutex::scoped_lock lock(command_mutex_);
    Twist output;

    // Get waypoint command command input
    if (waypoint_input_->connected() && waypoint_input_->enabled()) {
      updateWaypointCommand(waypoint_input_->getCommand());
    }

    // Get twist limit input
    if (twist_limit_input_->connected() && twist_limit_input_->enabled()) {
      twist_limit_ = twist_limit_input_->getCommand();
      twist_limit_valid_ = true;
    }

    // check command timeout
    // TODO

    // Check if pose control was preempted
    if (twist_output_->preempted()) {
      if (pose_command_valid_) {
        ROS_INFO_NAMED("position_controller", "Position control preempted!");
      }
      pose_command_valid_ = false;
    }

    // Check if motors are running
    if (motor_status_->motorStatus().running == false) {
      if (pose_command_valid_) {
        ROS_INFO_NAMED(
            "position_controller",
            "Disabled position control while motors are not running.");
      }
      pose_command_valid_ = false;
    }

    // Abort if no pose command is available
    if (!pose_command_valid_) {
      reset();
      twist_output_->stop();
      return;
    } else {
      twist_output_->start();
    }

    Pose pose = pose_->pose();
    //    Twist twist = twist_->twist();

    double yaw_command;
    {
      tf2::Quaternion q;
      double temp;
      tf2::fromMsg(waypoint_command_.pose.orientation, q);
      tf2::Matrix3x3(q).getRPY(temp, temp, yaw_command);
    }

    double yaw = pose_->getYaw();

    waypoint_command_.pose.position =
        position_limiter_(waypoint_command_.pose.position);

    output.linear.x = pid_.x.computeCommand(
        waypoint_command_.pose.position.x - pose.position.x, period);
    output.linear.y = pid_.y.computeCommand(
        waypoint_command_.pose.position.y - pose.position.y, period);
    output.linear.z = pid_.z.computeCommand(
        waypoint_command_.pose.position.z - pose.position.z, period);

    if (waypoint_command_.speed > 0) {
      geometry_msgs::Twist current_twist = twist_->twist();
      double current_speed = GetSpeed(current_twist.linear);

      const double speed_error = waypoint_command_.speed - current_speed;
      const double speed_command =
          pid_.speed.computeCommand(speed_error, period);

      output.linear.x = output.linear.x * speed_command;
      output.linear.y = output.linear.y * speed_command;
      output.linear.z = output.linear.z * speed_command;
    }

    double yaw_error = yaw_command - yaw;
    // detect wrap around pi and compensate
    if (yaw_error > M_PI) {
      yaw_error -= 2 * M_PI;
    } else if (yaw_error < -M_PI) {
      yaw_error += 2 * M_PI;
    }
    output.angular.z = pid_.yaw.computeCommand(yaw_error, period);

    // add twist command if available
    if (twist_input_->connected() && twist_input_->enabled()) {
      output.linear.x += twist_input_->getCommand().linear.x;
      output.linear.y += twist_input_->getCommand().linear.y;
      output.linear.z += twist_input_->getCommand().linear.z;
      output.angular.x += twist_input_->getCommand().angular.x;
      output.angular.y += twist_input_->getCommand().angular.y;
      output.angular.z += twist_input_->getCommand().angular.z;
    }

    // limit twist
    if (twist_limit_valid_) {
      double linear_xy = sqrt(output.linear.x * output.linear.x +
                              output.linear.y * output.linear.y);
      double limit_linear_xy =
          std::max(twist_limit_.linear.x, twist_limit_.linear.y);
      if (limit_linear_xy > 0.0 && linear_xy > limit_linear_xy) {
        output.linear.x *= limit_linear_xy / linear_xy;
        output.linear.y *= limit_linear_xy / linear_xy;
      }
      if (twist_limit_.linear.z > 0.0 &&
          fabs(output.linear.z) > twist_limit_.linear.z) {
        output.linear.z *= twist_limit_.linear.z / fabs(output.linear.z);
      }
      double angular_xy = sqrt(output.angular.x * output.angular.x +
                               output.angular.y * output.angular.y);
      double limit_angular_xy =
          std::max(twist_limit_.angular.x, twist_limit_.angular.y);
      if (limit_angular_xy > 0.0 && angular_xy > limit_angular_xy) {
        output.angular.x *= limit_angular_xy / angular_xy;
        output.angular.y *= limit_angular_xy / angular_xy;
      }
      if (twist_limit_.angular.z > 0.0 &&
          fabs(output.angular.z) > twist_limit_.angular.z) {
        output.angular.z *= twist_limit_.angular.z / fabs(output.angular.z);
      }
    }

    // set twist output
    twist_output_->setCommand(output);
  }

 private:
  void updateWaypointCommand(
      const hector_navigation_msgs::Waypoint &new_waypoint) {
    // TODO TF to world frame
    if (new_waypoint.header.frame_id != world_frame_) {
      ROS_WARN_STREAM_THROTTLE_NAMED(1.0, "waypoint_controller",
                                     "Waypoint commands must be given in the "
                                         << world_frame_
                                         << " frame, ignoring command");
    } else {
      updateWaypointCommand(new_waypoint.pose, new_waypoint.speed);
    }
  }

  void updateWaypointCommand(const geometry_msgs::Pose &new_pose,
                             const double &speed) {
    {
      waypoint_command_.pose.position = new_pose.position;
      // Strip non-yaw components from orientation
      tf2::Quaternion q;
      double roll, pitch, yaw;
      tf2::fromMsg(new_pose.orientation, q);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      q.setRPY(0, 0, yaw);
      waypoint_command_.pose.orientation = tf2::toMsg(q);
      waypoint_command_.speed = speed;
      pose_command_valid_ = true;
    }
    pose_marker_.pose = waypoint_command_.pose;
    marker_publisher_.publish(pose_marker_);
  }

  void initMarker(std::string name) {
    pose_marker_.header.frame_id = world_frame_;
    pose_marker_.ns = name;
    pose_marker_.id = 0;
    pose_marker_.type = visualization_msgs::Marker::ARROW;
    pose_marker_.scale.x = 0.15;
    pose_marker_.scale.y = pose_marker_.scale.z = 0.03;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.g = 0.5;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.a = 1.0;
  }

  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  MotorStatusHandlePtr motor_status_;

  WaypointCommandHandlePtr waypoint_input_;
  TwistCommandHandlePtr twist_input_;
  TwistCommandHandlePtr twist_limit_input_;
  TwistCommandHandlePtr twist_output_;

  hector_quadrotor_interface::PointLimiter position_limiter_;

  ros::Subscriber pose_subscriber_, twist_limit_subscriber_;
  ros::Publisher marker_publisher_;

  visualization_msgs::Marker pose_marker_;

  //   geometry_msgs::Pose pose_command_;
  hector_navigation_msgs::Waypoint waypoint_command_;
  geometry_msgs::Twist twist_limit_;
  bool pose_command_valid_, twist_limit_valid_;

  std::string base_link_frame_, base_stabilized_frame_, world_frame_;
  std::string tf_prefix_;

  struct {
    control_toolbox::Pid x, y, z, yaw, speed;
  } pid_;

  boost::mutex command_mutex_;
};

}  // namespace hector_navigation_controllers

PLUGINLIB_EXPORT_CLASS(hector_navigation_controllers::WaypointController,
                       controller_interface::ControllerBase)
