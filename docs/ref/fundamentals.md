# Fundamentals {#fundamentals}

Listed below are fundamental concepts behind the development of our SLAM
library. Note that this page is not to include external links, but mainly
summaries of concepts written by slam-core team members.


@todo The fundamentals page is incomplete.
More concepts will be added to this page as the project progresses.

## Mathematics
- @subpage math_se3
- [Nonlinear Least-Squares](ref/math/nlls.md)
- [Matrix Factorizations](ref/math/matrix.md)
- [Observability/Degeneracy](ref/math/observability.md)

## Kinematics
- [2D/3D Rigid Body Dynamics](ref/kinematics/rigidbody.md)
- [Simple Kinematic](ref/kinematics/kinematic.md)
- [Ground Vehicles](ref/kinematics/groundvehicles.md)
- [Aerial Vehicles](ref/kinematics/aerialvehicles.md)

## Vision Measurements
- @subpage vision_camera_models
- [Camera Calibration](ref/vision/calibration.md)
- @subpage vision_features
- @subpage vision_benchmarking

## Lidar Measurements
- [Lidar Models](ref/lidar/lidar_models.md)
- [Lidar Calibration](ref/lidar/calibration.md)
- @subpage lidar_scan

## Inertial Measurements
- [GPS Models](ref/inertial/gps_models.md)
- [Inertial Models](ref/inertial/inertial_models.md)
- [Allan Variance](ref/inertial/Allan Variance.md)

## Estimation
- [Filters](ref/estimation/filters.md)
- [Parity Space](ref/estimation/parity.md)
- [Integrity Monitoring](ref/estimation/integrity.md)

## Optimization
- @subpage backend_factor_graphs
