#pragma once

#include <hector_navigation_msgs/Waypoint.h>
#include <hector_quadrotor_interface/handles.h>

namespace hector_quadrotor_interface {

using hector_navigation_msgs::Waypoint;

class WaypointHandle : public Handle_<WaypointHandle, Waypoint> {
 public:
  using Base::operator=;

  WaypointHandle() : Base("waypoint") {}
  WaypointHandle(QuadrotorInterface *interface) : Base(interface, "waypoint") {}
  WaypointHandle(QuadrotorInterface *interface, const Waypoint *waypoint)
      : Base(interface, waypoint, "waypoint") {}

  virtual ~WaypointHandle() {}

  const ValueType &waypoint() const { return *get(); }

  void getEulerRPY(double &roll, double &pitch, double &yaw) const;
  double getYaw() const;
  double getSpeed() const;
  Vector3 toBody(const Vector3 &nav) const;
  Vector3 fromBody(const Vector3 &body) const;
};
typedef boost::shared_ptr<WaypointHandle> WaypointHandlePtr;

class WaypointCommandHandle
    : public CommandHandle_<WaypointCommandHandle, Waypoint> {
 public:
  using Base::operator=;

  WaypointCommandHandle() {}
  WaypointCommandHandle(QuadrotorInterface *interface, const std::string &name,
                        const std::string &field = std::string())
      : Base(interface, name, field) {}
  WaypointCommandHandle(Waypoint *command) { *this = command; }
  virtual ~WaypointCommandHandle() {}
};
typedef boost::shared_ptr<WaypointCommandHandle> WaypointCommandHandlePtr;

}  // namespace hector_quadrotor_interface
#include "waypoint_handle_impl.h"
