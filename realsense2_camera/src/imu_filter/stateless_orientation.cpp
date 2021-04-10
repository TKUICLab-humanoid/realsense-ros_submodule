
#include "imu_filter/stateless_orientation.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

template<typename T>
static inline void crossProduct(
      T ax, T ay, T az,
      T bx, T by, T bz,
      T& rx, T& ry, T& rz) {
  rx = ay*bz - az*by;
  ry = az*bx - ax*bz;
  rz = ax*by - ay*bx;
}


template<typename T>
static inline T normalizeVector(T& vx, T& vy, T& vz) {
  T norm = sqrt(vx*vx + vy*vy + vz*vz);
  T inv = 1.0 / norm;
  vx *= inv;
  vy *= inv;
  vz *= inv;
  return norm;

}

bool StatelessOrientation::computeOrientation(
  geometry_msgs::Vector3 A,
  geometry_msgs::Vector3 E,
  geometry_msgs::Quaternion& orientation) {

  float Hx, Hy, Hz;
  float Mx, My, Mz;
  float normH;

  // A: pointing up
  float Ax = A.x, Ay = A.y, Az = A.z;

  // E: pointing down/north
  float Ex = E.x, Ey = E.y, Ez = E.z;

  // H: vector horizontal, pointing east
  // H = E x A
  crossProduct(Ex, Ey, Ez, Ax, Ay, Az, Hx, Hy, Hz);

  // normalize H
  normH = normalizeVector(Hx, Hy, Hz);
  if (normH < 1E-7) {
    // device is close to free fall (or in space?), or close to
    // magnetic north pole.
    // mag in T => Threshold 1E-7, typical values are  > 1E-5.
    return false;
  }

  // normalize A
  normalizeVector(Ax, Ay, Az);

  // M: vector horizontal, pointing north
  // M = A x H
  crossProduct(Ax, Ay, Az, Hx, Hy, Hz, Mx, My, Mz);

  // Create matrix for basis transformation
  tf2::Matrix3x3 R;
  
  R[0][0] = Mx;     R[0][1] = Hx;     R[0][2] = -Ax;
  R[1][0] = My;     R[1][1] = Hy;     R[1][2] = -Ay;
  R[2][0] = Mz;     R[2][1] = Hz;     R[2][2] = -Az;
  
  // Matrix.getRotation assumes vector rotation, but we're using
  // coordinate systems. Thus negate rotation angle (inverse).
  tf2::Quaternion q;
  R.getRotation(q);
  tf2::convert(q.inverse(), orientation);
  return true;
}


bool StatelessOrientation::computeOrientation(
  geometry_msgs::Vector3 A,
  geometry_msgs::Quaternion& orientation) {

  // This implementation could be optimized regarding speed.

  // magnetic Field E must not be parallel to A,
  // choose an arbitrary orthogonal vector
  geometry_msgs::Vector3 E;
  if (fabs(A.x) > 0.1 || fabs(A.y) > 0.1) {
      E.x = A.y;
      E.y = A.x;
      E.z = 0.0;
  } else if (fabs(A.z) > 0.1) {
      E.x = 0.0;
      E.y = A.z;
      E.z = A.y;
  } else {
      // free fall
      return false;
  }

  return computeOrientation(A, E, orientation);
}
