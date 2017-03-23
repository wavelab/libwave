# wave.kinematics.rotation

Our rotation library component internally uses the Kindr library's rotation class to perform elementary operations. For more information on the Kindr library rotations, see their documentation [here](http://docs.leggedrobotics.com/kindr/page_rotations.html).

**Class member variables**

- - - -

Internal class storage.  The rotation object of our class internally uses a double precision kindr RotationMatrix.

```
kindr::RotationMatrixD rotation_object;
```
- - - -

The threshold that is used to compare if rotation matrices are sufficiently close to one another.  see isNear() function for more details.
```
static double comparision_threshold;
```
- - - -

**Class Member Functions**:

## Rotation()
Default constructor for a rotation object.  By default, it is initialized to Identity.

## static bool fromEulerXYZ(const double rotation_x, const double rotation_y, const double rotation_z, Rotation &R);
Creates a rotation object using a X-Y-Z Euler rotation order, such that
\[
R = R_z R_y R_x,
\]
where $R_x$,$R_y$, and $R_z$, are rotation matrices constructed using the rotation about $x$, $y$, and $z$, respectively. 

```
Input:
  rotation_x (double): Rotation about x-axis in radians.
  rotation_y (double): Rotation about y-axis in radians.
  rotation_z (double): Rotation about z-axis in radians.
Output:
  R (wave::Rotation): Rotation object constructed using the supplied input.
```
  
## static bool fromAngleAxis(const double angle_magnitude, const Vec3 rotation_axis, Rotation &R)
Creates a rotation object from an axis-angle representation.

```
Input:
  angle_magnitude (double): Magnitude of the rotation;
  rotation_axis (wave::Vec3): 3D vector corresponding to the axis of rotation.
Output:
  R (wave::Rotation): Rotation object constructed using the supplied input.
```

## static bool fromExpMap(const Vec3 se3_vector, Rotation &R);
Creates a rotation object from the so(3) Lie algebra, to the SO(3) Lie group using the exponential map.
\[
R = exp(w_{\times}),
\]
where $w_{\times}$ is the input se3_vector represented in skew-symmetric form.

```
Input:
  se3_vector (wave::Vec3): Vector corresponding to the se3 lie algebra for the rotation.
Output:
  R (wave::Rotation): Rotation object constructed using the supplied input.
```

## wave::Vec3 logMap()
Returns the se3 Lie algebra parameters for ```this``` rotation,
\[
w = log(R)_{\vee},
\]
where $\vee$ converts the skew-symmetric form of the Lie algebra parameters $w_{\times}$ to a the 3D vector $w \in \mathbb{R}^3$ 

```
Input:
  none
Returns:
  (wave::Vec3): output of logarithmic map for  ```this``` rotation.
```

## void setToIdentity()
Sets ```this``` rotation to Identity.


## Vec3 rotate(const Vec3 &input_vector)
Rotates the input vector by ```this``` Rotation,
\[
\label{eqn:rot}
p_o = R p_i,
\]
where $p_i$ is the input vector, and $p_o$ is the rotated output vector.


```
Input:
  input_vector (wave::Vec3): 3D input vector to be rotated.
Returns:
  (wave::Vec3): Output vector which is the input vector rotated by ```this``` rotation.
```
## Vec3 rotateAndJacobian(const Vec3 &input_vector, Mat3 &Jpoint, Mat3 &Jparam)
Same as the rotate() function, except also computes the Jacobians of Eq $\ref{eqn:rot}$ with respect to both the input point, $p_i$, and ```this``` rotation, $R$.

```
Input:
  input_vector (wave::Vec3): 3D input vector to be rotated.
Output:
  (wave::Mat3): Jacobian of rotation function with respect to the point input.
  (wave::Mat3): Jacobian of rotation function with respect to ```this``` rotation.
```

## Vec3 inverseRotate(const Vec3 &input_vector)
Rotates the input vector by the inverse of ```this``` rotation,
\[
p_o = R^{-1} p_i,
\]
where $p_i$ is the input vector, and $p_o$ is the output vector rotated by the the inverse of ```this``` rotation.
```
Input:
  input_vector (wave::Vec3): 3D input vector to be inverse rotated.
Returns:
  (wave::Mat3): Output vector which is the input vector rotated by the inverse of ```this``` rotation.
```

## void compose(const Rotation &R)
Modifies ```this``` rotation in-place, such that it becomes the **right** composition of ```this``` rotation and the input rotation,
\[
R_t = R_t R,
\]
where $R_t$ is ```this``` rotation, and $R$ is the input rotation.
```
Input:
  input_vector (wave::Rotation): Rotation object to be composed
Returns:
  none.
```

## Mat3 invert()
Modifies ```this``` rotation in-place, to become the inverse of ```this``` rotation,
\[
R_t = R_t^{-1},
\]
where $R_t$ is ```this``` rotation.


## isNear(const Rotation &R);
Checks to see if the input rotation is sufficiently close to ```this``` rotation, such that they can be considered equal.  The function computes a disparity rotation as,
\[
R_d = R_t R^{-1}.
\]
If the rotations are equal, $R_d$ will be the identity matrix, or equivalently, the log map of $R_d$ will be zero,

\[
w_d = log(R_d).
\]
The input rotation is considered sufficiently close to ```this``` rotation if,
\[
||w_d|| \leq \epsilon,
\]
where $\epsilon$ is set by wave::Rotation::comparision_threshold.

## manifoldPlus(const Vec3 omega);
Updates ```this``` rotation in-place, according to the manifold-plus operation,
\[
R_t = R_t \boxplus \omega,
\]
where $ R_t$ is ```this``` rotation, $\omega \in \mathbb{R}^3$ is a vector in the tangent space of $R_t$, and the $\boxplus$ operator implements the following **left** composition using the exponential map,

\[
\boxplus: \mathbb{SO}(3) \times \mathbb{R}^3 \mapsto \mathbb{SO}(3) \\
R_t = exp(\omega)R_t.
\]

```
Input:
  input_vector (wave::Vec3): 3D vector in the tangent space of ```this``` rotation.
Returns:
  none.
```
 
## Mat3 getRotationMatrix()
Returns the rotation matrix corresponding to ```this``` rotation.

```
Input:
  none.
Returns:
  (wave::Mat3): 3x3 rotation matrix for ```this``` rotation.
```