#include "waypoint_handle.h"

namespace hector_quadrotor_interface {

void WaypointHandle::getEulerRPY(double &roll, double &pitch,
                                 double &yaw) const {
  const Quaternion::_w_type &w = waypoint().pose.orientation.w;
  const Quaternion::_x_type &x = waypoint().pose.orientation.x;
  const Quaternion::_y_type &y = waypoint().pose.orientation.y;
  const Quaternion::_z_type &z = waypoint().pose.orientation.z;
  roll = atan2(2. * y * z + 2. * w * x, z * z - y * y - x * x + w * w);
  pitch = -asin(2. * x * z - 2. * w * y);
  yaw = atan2(2. * x * y + 2. * w * z, x * x + w * w - z * z - y * y);
}

double WaypointHandle::getYaw() const {
  const Quaternion::_w_type &w = waypoint().pose.orientation.w;
  const Quaternion::_x_type &x = waypoint().pose.orientation.x;
  const Quaternion::_y_type &y = waypoint().pose.orientation.y;
  const Quaternion::_z_type &z = waypoint().pose.orientation.z;
  return atan2(2. * x * y + 2. * w * z, x * x + w * w - z * z - y * y);
}

double WaypointHandle::getSpeed() const {
  const double speed = waypoint().speed;
  return speed;
}

Vector3 WaypointHandle::toBody(const Vector3 &nav) const {
  const Quaternion::_w_type &w = waypoint().pose.orientation.w;
  const Quaternion::_x_type &x = waypoint().pose.orientation.x;
  const Quaternion::_y_type &y = waypoint().pose.orientation.y;
  const Quaternion::_z_type &z = waypoint().pose.orientation.z;
  Vector3 body;
  body.x = (w * w + x * x - y * y - z * z) * nav.x +
           (2. * x * y + 2. * w * z) * nav.y +
           (2. * x * z - 2. * w * y) * nav.z;
  body.y = (2. * x * y - 2. * w * z) * nav.x +
           (w * w - x * x + y * y - z * z) * nav.y +
           (2. * y * z + 2. * w * x) * nav.z;
  body.z = (2. * x * z + 2. * w * y) * nav.x +
           (2. * y * z - 2. * w * x) * nav.y +
           (w * w - x * x - y * y + z * z) * nav.z;
  return body;
}

Vector3 WaypointHandle::fromBody(const Vector3 &body) const {
  const Quaternion::_w_type &w = waypoint().pose.orientation.w;
  const Quaternion::_x_type &x = waypoint().pose.orientation.x;
  const Quaternion::_y_type &y = waypoint().pose.orientation.y;
  const Quaternion::_z_type &z = waypoint().pose.orientation.z;
  Vector3 nav;
  nav.x = (w * w + x * x - y * y - z * z) * body.x +
          (2. * x * y - 2. * w * z) * body.y +
          (2. * x * z + 2. * w * y) * body.z;
  nav.y = (2. * x * y + 2. * w * z) * body.x +
          (w * w - x * x + y * y - z * z) * body.y +
          (2. * y * z - 2. * w * x) * body.z;
  nav.z = (2. * x * z - 2. * w * y) * body.x +
          (2. * y * z + 2. * w * x) * body.y +
          (w * w - x * x - y * y + z * z) * body.z;
  return nav;
}

}  // namespace hector_quadrotor_interface
