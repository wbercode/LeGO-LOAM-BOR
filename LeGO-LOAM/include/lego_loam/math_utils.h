#ifndef LEGO_LOAM_MATH_UTILS_H
#define LEGO_LOAM_MATH_UTILS_H

#include <Eigen/Geometry>

typedef Eigen::Vector3f Vector3;

struct RollPitchYaw{
  double roll;
  double pitch;
  double yaw;
  RollPitchYaw():roll(0),pitch(0),yaw(0) {}
};

struct Transform
{
  Transform():pos(Vector3::Zero()) {}
  Vector3 pos;
  RollPitchYaw rot;
};


inline Vector3 ApplyRotationRoll(const Vector3& p, double roll)
{
  double Cos = cos(roll);
  double Sin = sin(roll);
  return {p.x(),
          Cos * p.y() - Sin* p.z(),
          Sin * p.y() + Cos* p.z() };
}

inline Vector3 ApplyRotationPitch(const Vector3& p, double pitch)
{
  double Cos = cos(pitch);
  double Sin = sin(pitch);
  return { Cos * p.x() + Sin* p.z(),
          p.y(),
          -Sin * p.x() + Cos* p.z()
  };
}

inline Vector3 ApplyRotationYaw(const Vector3& p, double yaw)
{
  double Cos = cos(yaw);
  double Sin = sin(yaw);
  return { Cos * p.x() - Sin* p.y(),
          Sin * p.x() + Cos* p.y(),
          p.z() };
}

inline Eigen::Affine3f TransformZXYT(const Transform& tr)
{
  return Eigen::Translation3f(tr.pos) *
         Eigen::AngleAxisf(tr.rot.yaw, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(tr.rot.roll, Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxisf(tr.rot.pitch, Eigen::Vector3f::UnitY());
}

#endif // LEGO_LOAM_MATH_UTILS_H
