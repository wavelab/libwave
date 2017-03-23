# wave.kinematics.rotation

**Class member functions**

<code>
kindr::RotationMatrixD rotation_object;
static double comparision_threshold;
</code>

**Functions**:
  - Rotation();
  - static bool fromEulerXYZ(const double rotation_x,
                           const double rotation_y,
                           const double rotation_z,
                           Rotation &R);
  
  - static bool fromAngleAxis(const double angle_magnitude,
                            const Vec3 rotation_axis,
                            Rotation &R);
  - static bool fromExpMap(const Vec3 se3_vector, Rotation &R);


  - Vec3 logMap();
  - void setToIdentity();
  - Vec3 rotate(const Vec3 &input_vector);
  - Vec3 rotateAndJacobian(const Vec3 &input_vector,
                         Mat3 &Jpoint,
                         Mat3 &Jparam);
  - Vec3 inverseRotate(const Vec3 &input_vector);
  - void compose(const Rotation &R);
  - Mat3 invert();
  - bool isNear(const Rotation &R);
  - void manifoldPlus(const Vec3 omega);

 
  - Mat3 getRotationMatrix();
};