#ifndef __WAVE_KINEMATICS_DATA_HPP__
#define __WAVE_KINEMATICS_DATA_HPP__

#include "wave/utils/math.hpp"
#include "wave/utils/utils.hpp"


namespace wave {

class Pose {
public:
  Quaternion q;
  Vec3 position;

  Pose(void);
  Pose(Quaternion q, Vec3 position);
  Pose(double roll, double pitch, double yaw, double x, double y, double z);
  Mat3 rotationMatrix(void);
  void printPosition(void);
  void printOrientation(void);
  void printQuaternion(void);
  void print(void);
};

}  // end of wave namespace
#endif
