#pragma once

// Mahony's explicit complementary filter (IMU only, no magnetometer).
// The quaternion takes a vector in the sensor frame into the world frame:
//   v_world = R(q) * v_sensor
// World +Z is up, matching OpenVINS (gravity acceleration is -Z, so a
// stationary accelerometer reads the upward specific force).

#include <cmath>

namespace argus_mahoney {

struct MahonyFilter {
  // w, x, y, z
  double q[4] = {1.0, 0.0, 0.0, 0.0};
  // Proportional and integral gains on the accelerometer error (rad/s per unit error).
  double kp = 0.5;
  double ki = 0.0;
  double integral[3] = {0.0, 0.0, 0.0};

  static void normalize(double q[4]) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-12) {
      q[0] = 1.0;
      q[1] = q[2] = q[3] = 0.0;
      return;
    }
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
  }

  // Roll and pitch from the measured specific force. Yaw stays zero.
  // A stationary accelerometer points up.
  void set_tilt_from_accel(double ax, double ay, double az) {
    const double n = std::sqrt(ax * ax + ay * ay + az * az);
    if (n < 1e-6) {
      return;
    }
    const double roll = std::atan2(ay, az);
    const double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
    normalize(q);
    integral[0] = integral[1] = integral[2] = 0.0;
  }

  // World +Z expressed in the sensor frame, R(q)^T * [0, 0, 1].
  void up_in_sensor(double &vx, double &vy, double &vz) const {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    vx = 2.0 * (x * z - w * y);
    vy = 2.0 * (w * x + y * z);
    vz = w * w - x * x - y * y + z * z;
  }

  // gx, gy, gz in rad/s, already bias-corrected. ax, ay, az in any units.
  // dt in seconds. Samples with a non-positive or very large dt are ignored.
  void update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
    if (!(dt > 0.0) || dt > 0.2) {
      return;
    }
    const double n = std::sqrt(ax * ax + ay * ay + az * az);
    if (n > 1e-6) {
      ax /= n;
      ay /= n;
      az /= n;
      double vx, vy, vz;
      up_in_sensor(vx, vy, vz);
      const double ex = ay * vz - az * vy;
      const double ey = az * vx - ax * vz;
      const double ez = ax * vy - ay * vx;
      if (ki > 0.0) {
        integral[0] += ki * ex * dt;
        integral[1] += ki * ey * dt;
        integral[2] += ki * ez * dt;
        gx += integral[0];
        gy += integral[1];
        gz += integral[2];
      }
      gx += kp * ex;
      gy += kp * ey;
      gz += kp * ez;
    }

    const double ox = gx * dt;
    const double oy = gy * dt;
    const double oz = gz * dt;
    const double angle = std::sqrt(ox * ox + oy * oy + oz * oz);
    double dw, dx, dy, dz;
    if (angle < 1e-8) {
      dw = 1.0;
      dx = ox * 0.5;
      dy = oy * 0.5;
      dz = oz * 0.5;
    } else {
      const double s = std::sin(angle * 0.5) / angle;
      dw = std::cos(angle * 0.5);
      dx = ox * s;
      dy = oy * s;
      dz = oz * s;
    }
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    q[0] = w * dw - x * dx - y * dy - z * dz;
    q[1] = w * dx + x * dw + y * dz - z * dy;
    q[2] = w * dy - x * dz + y * dw + z * dx;
    q[3] = w * dz + x * dy - y * dx + z * dw;
    normalize(q);
  }
};

}  // namespace argus_mahoney
