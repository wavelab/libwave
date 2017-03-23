# wave.kinematics

This module contains methods to perform kinematics on SO(3) and SE(3) manifolds. Jacobians are also computed for many of the expressions, which are necessariy to perform estimation and optimization of the manifold quantities.

Our kinematics library interally uses the Kindr library to perform elementary operations.  For more information on the Kindr Library, see their documentation [here](http://docs.leggedrobotics.com/kindr/).


Much of the differential calculus used to derive the Jacobian quantities is based upon [A Primer on the Differential Calculus of 3D Orientations](https://arxiv.org/find/cs/1/au:+Sommer_H/0/1/0/all/0/1).