# Open Source Package Evaluation {#reading_open_source}

## Calibration

### [Kalibr](https://github.com/ethz-asl/kalibr/wiki)

License: BSD

#### Validator: Arun

Work in progress, see: [[Kalibr Test Log]]

#### Steve's Comments

Kalibr is actually three separate calibration packages, and new features are still being added

* Camera-IMU calibration, the original and what Kalibr is known for, with continuous time batch estimation that calibrates spatial (extrinsic) and temporal parameters.
* Multi-camera calibration, requiring overlap between pairs of cameras but operating in pre-mapped outdoor environments without targets
* A rolling shutter parameter calibrator.
* Experimental includes camera-imu-lidar code from a 2016 journal paper.

##### Camera-IMU calibration

The Camera-IMU calibration takes Furgale's work from UTIAS with Tim Barfoot on Continuous time SLAM and applies it to the vision IMU calibration process.  The advantage of working in the continuous domain is that it is possible to shift the alignment of measurements during the calibration process, and therefore simultaneously calibrate both the extrinsic transformation between camera and IMU, and the time offset between measurements.  The checkerboard is left static and the camera-IMU pair are moved with large, aggressive motions near the target, exciting the accelerometers and gyroscopes in the IMU.

The calibration process involves estimating the following quantities
* Static gravity vector
* Static transform between camera frame and IMU frame
* Static offset between camera measurement and IMU measurements
* Dynamic Pose trajectory of IMU
* Dynamic gyroscope and accelerometer biases

The pose states are defined as parametrized b-splines, for which velocity and acceleration equations can be found through differentiation.  The biases are modeled as a random walk, i.e. driven by Gaussian noise. The optimization is performed over small batches of measurements, and uses weighted quadratic error terms on the innovation for visual feature measurements, accelerometer and gyroscope measurements.  Since the target size is known, the map points are fixed in the environment, and related to the IMU frame through the gravity vector estimate.  This ensures scale is resolved on the image measurements.

This is a very nice approach, in that it simultaneously resolves temporal and spatial calibration.  It suffers from a discontinuity at knot points on the splines (the points that parametrize the spline curvature). If a measurement slips from one side of a knot point to another, it can lead to instability in the optimization, and numerical inaccuracy.  2016 work extends kalibr to multi-IMU.



##### Multi-camera calibration

In addition to the continuous time batch estimation process, kalibr also contains a process in which it evaluates the data collected and the motions observed, and computes a numerical observability of the parameters to ensure parameters are not updated based on noise alone.  Degenerate motions are possible, which do not sufficiently excite the modes of the calibration optimization and lead to parameters being updated based on noise in the measurements, even though they are really unobservable in the motions captured.  The formulation for this aspect of the calibration toolbox does not match the continuous-time batch optimization, it appears to be discrete with known temporal offset.  The requirement for overlap between cameras resolves scale for them without the IMU.


##### Rolling shutter calibration

Not yet reviewed, but looks reasonable.


### [CamOdoCal](https://github.com/hengli/camodocal)

Reviewer: Steve
License: CC-BY-SA, not quite as bad as GPL, but all changes to their stuff must be shared.  

ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original. [CC License] (https://creativecommons.org/licenses/by-sa/2.0/)

CamOdoCal, is, as the name suggests, a calibration toolkit for cameras and wheel odometry. The target is clearly multi-camera installations on cars, and it performs intrinsic and extrinsic calibration in the following steps.

1. Checkerboard based intrinsic calibration of each camera. The camera models include a) Pinhole camera model, b)
Unified projection model (C. Mei, and P. Rives, Single View Point Omnidirectional Camera Calibration from Planar Grids, ICRA 2007) and c) Equidistant fish-eye model (J. Kannala, and S. Brandt, A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses, PAMI 2006).  They don't like Scaramuzza, as they think they can't get Jacobians for it (tbd).

2. They then run  monocular VO with sliding window bundle adjustment for each camera, from driving around and collecting synchronized images and odometry measurements.  The trajectories are slow and curved, and each camera ultimately generates its own scale and transform to the odometry measurement point.

3. They then triangulate feature correspondences in individual camera maps and in 3D using the monocular VO solution, and perform bundle adjustment to improve each map.  The actual poses are held fixed, and only the feature points and camera odometry transformations are optimized.

4. Then they go for full-scale map fusion and look for feature correspondences between cameras in the separate maps.  They have no real-time constraint though, so they rectify image pairs to a common viewpoint (not just the patches around features as in MCPTAM), and search for all corresponding points between maps painstakingly.  Loop closures are also detected using a vocabulary tree (dbow2) over all images from all cameras allowing further correspondences.  Finally,  they run another bundle adjustment similar to that described earlier, but which also optimizes all intrinsic camera parameters and odometry poses, to obtain a set of intrinsic parameters for each camera, a set of camera-odometry transforms, and a globally- consistent sparse map of landmarks. I'm amazed this is observable, but likely only because of the extensive initialization and the quality of those early estimates for all the parameters.

The end result takes 2+ hours to run for their demo sequence in a highly structured and feature rich urban area.  It seems like point-cloud registration could be used once the maps are defined, as long as the uncertainty directions (the depth) is understood.  Also, the videos show a huge amount of clutter and bad points, I wonder if fewer, better points (surrogate target corners) would work more reliably.


The code relies on a lot of common elements, may be worth a look. BLAS, Boost, Cuda, Eigen3, OpenCV, SuiteSparse, Ceres, (glog, OpenMP too).   Looks like pure OpenCV feature detection, description and matching. Reminded me of Jason's code.  Nicely decomposed VO though, temporal, sliding window, could be a helpful readthrough before we code up our own. Finished in 2014, it's been maintained and should install on 16.05 kinetic. Recent commits.

## SLAM

### [ORB SLAM](https://github.com/raulmur/ORB_SLAM2)

Validator: Jason

Licence: GPLv3

[Technical Summary slides](https://github.com/wavelab/wavelab.github.io/blob/master/documents/slam-core/ORBSLAM%20summary.pdf)

## Visual Odometry



### [SVO](https://github.com/uzh-rpg/rpg_svo)

Validator: Arun, Stan

Licence: GPLv3
Ranking on documentation, code quality, implementation, etc.

#### Arun's Comments:
 Tried in room with limited lighting and rolling shutter camera.  Initial tests show poor results, as initialization failed readily, and tracking was lost with only modest angular rate. Additional testing with global shutter camera and various environmental conditions required.

#### Stan's Comments:
I tried it a few weeks ago with a Ximea camera and a 120 FOV lens. Calibrated it using PTAM's ATAN calibration tool as they recommend in the documentation. SVO seems to work with moving forward and backward motions only. Any rotation lead to it it failing. Despite hours of effort at the time, I never once saw the edge lets they add to the scene in the youtube videos and discussed in the paper. I suspect it is related to some of the parameters you can set when launching program, but there are over 30 of those.

Installation: Bit of a pain. Most ETH packages I find more difficult then required to install. Usually require extra packages and often you need to alter the CMAKE files as they do not test them on other machines. If you have errors related to Eigen, just remove the line find_package(Eigen) as Eigen is just a header library and you only need to include that directory.

On a side note, it appears that you can add imu information if you provide a calibration file kalibre. Perhaps we should look at making our own bags? If provided a camera and an IMU, I can attach it to my quadrotor and collect a few aerial data sets.   

#### Chris's comments
From a code perspective there are some external components worth mentioning:

-  `vikit`: vision utility library containing generic C++ classes for pinhole camera model, atan camera, omni camera model, calculations for homography. Some of the code are GPL code from PTAM, som from DVO (Dense Visual Slam from TUM university), but it has some interesting SSE2 SSE3 optimizations for calculating Shi-Tomasi score and others worth looking at.
- The bundle adjuster was using `g2o` for the optimization
- Front end fast corner detection was using the Edward Rosten's original Fast library code.
- Very slow to close issues, they currently have 118 opened isssues 4 within the last month, compared to only 1 closed in the same period.

- Semi-direct monocular VO
- Operates directly on pixel intensities rather than features
- Probabilistic mapping method
- 55 fps on Odroid U2 (ARM 1.6Ghz), 300 fps on laptop (Intel i7, 8 cores,
  2.8Ghz)
- Did not have metric comparing the PTAM vs SVO to ground truth, alot of the
  focus was just saying we are faster than PTAM and had plots to show it
  tracked the ground truth better.

**Direct methods**: estimate structure and motion directly from intensity
  values in the image. The local intensity gradient magnitude and direction is
  used in the optimization compared to feature based methods that consider
  only distance to some feature location. Direct methods that exploit all
  the information in the image, even from areas where gradients are small
  have been shown to outperform feature-based methods in terms of
  robustness in scenes with little texture or in the case of camera
  defocus and motion blur. The computation of the **photometric error** is
  more intensive than the reprojection error, as it involves warping and
  integrating large image regions. However, since direct methods operate
  directly on the intensity values of the image, the time for feature
  detection and invariant descriptor computation can be saved.


### [ROVIO](https://github.com/ethz-asl/rovio)

Validator: Stan

Licence: BSD (just not labelled as such)

Ranking on documentation, code quality, implementation, etc.

The code looks promising based on the videos / paper. However, installing is tedious, again you must follow a block of instructions and add extra packages on the way that are not directly stated in the wiki. (This is why install scripts + travis are great). Once you have done that, launch the code is straight forward.

### Running on bags:
When run on the provided bags the code starts to be a disappointment. Even on the bags, the code clearly does not work in real time with the provided parameters from the author. So either the parameters are wrong, or it is post processed in some way. I tried several bags and had the same issue. The solver took to long on each frame to provide a reliable estimate and it clearly did not work in real time. Skimming the paper there was no mention of frame rate, or recommended image size, or the computer that was used to test the code / create the video.

### Running live:
I also attempted to run the code live using a point grey chameleon 3 with a 120 fov lens and a MPU6500 imu connected through the gimbal. I also had similar findings to the bag data. Further, feature tuning is likely required to get better results as I feel that this package was tuned for aerial vehicles and not indoor environments.


## [LIBVISO2](http://www.cvlibs.net/software/libviso/)
See [LibVISO2 Summary](LibVISO2-summary)

## Other visual odometry libraries

#### Lists of libraries
* https://github.com/openMVG/awesome_3DReconstruction_list
* https://github.com/tzutalin/awesome-visual-slam

#### Non-GPL
* [bvpo](https://github.com/halismai/bpvo) Uses eigen, OpenCV, C++11. LGPL. Uses raw intensity for real-time speed, or [bit-planes](https://arxiv.org/abs/1604.00990) for low light conditions. Worth looking into?
* [mono-vo](https://github.com/avisingh599/mono-vo) MIT license. The one in [Avi Singh's blog post](http://avisingh599.github.io/vision/monocular-vo/). "Uses Nister's Five Point Algorithm for Essential Matrix estimation, and FAST features, with a KLT tracker."
* [Photoconsistency VO](https://github.com/mylxiaoyi/photoconsistency-visual-odometry). BSD. "Photoconsistency error." 5 years old.

#### GPL
* [libfovis](https://github.com/fovis/fovis) Uses depth info as input (e.g. Kinect)
* [rebvo](https://github.com/JuanTarrio/rebvo) "Realtime Edge Based Visual Odometry for a Monocular Camera"
* [dso (Direct Sparse Odometry)](https://github.com/JakobEngel/dso) ([Paper](https://arxiv.org/abs/1607.02565)) Featureless, direct, monocular.
* [rpg_svo](https://github.com/uzh-rpg/rpg_svo) Semi-direct monocular visual odometry. [2014 ICRA paper](https://github.com/uzh-rpg/rpg_svo/blob/master/Forster2014ICRA.pdf)
* The famous [LSD-SLAM](https://github.com/tum-vision/lsd_slam)
* [ORB SLAM](https://github.com/raulmur/ORB_SLAM)

## [Laser SLAM](https://github.com/wavelab/neptec_laser_slam)

Validator: Arun

Licence: ?

Laser SLAM is a package that was initially developed for the NASA Sample return challenge, where it was used to perform 3D localization using our 32 beam Velodyne:

<div style="position:relative;height:0;padding-bottom:56.25%"><iframe src="https://www.youtube.com/embed/PUmX5g_AHIs?ecver=2" width="640" height="360" frameborder="0" style="position:absolute;width:100%;height:100%;left:0" allowfullscreen></iframe></div>


Since then we have also used it for planetary mapping:

<div style="position:relative;height:0;padding-bottom:56.25%"><iframe src="https://www.youtube.com/embed/LVV88RZq0YQ?ecver=2" width="640" height="360" frameborder="0" style="position:absolute;width:100%;height:100%;left:0" allowfullscreen></iframe></div>


More recently, we have used the package to create off-line localization maps for self driving:

<div style="position:relative;height:0;padding-bottom:56.25%"><iframe src="https://www.youtube.com/embed/1-e2eD8y4E8?ecver=2" width="640" height="360" frameborder="0" style="position:absolute;width:100%;height:100%;left:0" allowfullscreen></iframe></div>


### Technical Summary
The Laser SLAM code is split into two processes: A scan server which loads laser scans from a source (bag file, csv, etc), and feeds them to the second part, the mapper, which performs scan registration and graph optimization to construct the map.  The scan server is fairly simple, thus our focus will be on the operation of the mapper.  Note that the package does not yet perform motion compensation as part of the optimization pipeline at this time, so motion compensation must be performed prior to sending the scans to the mapper.

At a high level, the mapper works by inserting each scan as a key-frame into a pose graph optimization.  Residuals between key-frames associated with overlapping scans are computed using ICP, which are then fed to a nonlinear least squares optimizer. The residuals in this case are simply the error between nodes and their relative pose estimates, similar to an odometry residual.  At each insertion, the pose graph is optimized.  By aggregating the scans from their optimized poses, the high quality, point-cloud based map is formed.  

Upon receiving a scan and its initialization pose, in the local map frame, from the server, the mapper performs the following steps:
*  Preprocessing is performed on the scan, which at this time is Gaussian Process based ground segmentation.  See [here](https://uwspace.uwaterloo.ca/bitstream/handle/10012/7431/Das_Arun_2013.pdf) for more details on how the ground segmentation is performed.
* The scan is transformed into the local map frame using the initialization pose.
* From the initialization pose, the K nearest neighbor key-frames are found, then:
* (1) Using an initial voxel based down-sampling, ICP is performed between the down sampled scans of the K nearest neighbours key-frames, generating residuals (graph edges) between these estimation states.  The associated information matrix for each residual is computed using the Lu and Milios (LUM) [method](http://kos.informatik.uni-osnabrueck.de/download/ras2007.pdf)
* (2) Once the residuals are added to the pose graph, the graph optimization is performed. Currently, the code supports the g2o and GTSAM back-ends.
* The above steps of (1) and (2) are repeated for M iterations, with successively smaller down sampling voxels.  The idea is to perform a course-to-fine optimization, that avoids local minima for the scan registration and allows for recovery of modest initialization pose errors.
*  The above steps are repeated for all scans inserted into the pose graph.
* Note that the aggregated point cloud can be optionally converted into a multi level surface (MLS) representation, which allows for obstacle and drivability analysis of the point cloud.


### Future Work
We've identified the following improvements for this code:
* Better modularity, should be improved once once absorbed into libwave
* Residual information matrix uses the LUM approach, which is specifically for ICP cost function.  We should look to generalize for NDT, GICP, etc.
* Inclusion of kinematic models / pre-integrated IMU measurements
* Add additional laser based feature/landmark states
* Strategies for improving the basin of convergence of the scan registrations.

## Laser Odometry and Mapping [LOAM](https://github.com/laboshinl/loam_velodyne)

Validator: Ben

License: Original version was released under BSD. Newer versions are commercial software.

### Technical summary ([paper](http://www.roboticsproceedings.org/rss10/p07.pdf))

At a high level, this package performs scan matching using LIDAR features to recover motion. It runs as three processes, each of which feeds the next process.
* The scan registration process is responsible for extracting features from the current scan and integrating IMU measurements (no filter though) to account for changing velocity during the scan. It provides the integrated IMU transformation and feature pointclouds to the...
* Laser odometry process. This process performs scan matching between the current features and the last set of features. It then uses the recovered velocity from scan matching to provide a motion corrected pointcloud and features to the...
* Laser mapping process. This is very similar to the previous process except that the pointcloud has already been motion corrected and that more features are extracted for better results. In the original application, this process runs at 0.1x the rate of the odometry node, although performance requirements are low enough that it could run at the same speed. This process provides an aggregate pointcloud map.

#### Features
The features used by this method are extracted using a smoothness term. The smoothness term is defined for each point in the scan, and is the norm of the sum of all vectors from close by points to the point of interest divided by the number of points and the norm of the distance from the lidar to the point of interest. The number of close-by points is configurable, and they are selected to be from the same scan line due to the anisotropic resolution of LIDAR. The scores should not change based on perspective (provided nothing is occluded) but there is some scaling based on inverse distance to that point.

### Process
* Store IMU data while a scan is in progress.
* One a scan is received, transform it for non-linear motion using IMU data, and extract features from each scan line. The scan lines are broken into subregions to distribute the features over the entire scan. Using the smoothness criteria, edge points are selected as high-scoring points and planar areas selected as low scoring points.
* Take the extracted features and match them against the closest features in the previous scans, using the initial estimate of the motion between scans to transform them (provided by integrating the IMU and adding the previous scan's velocity)
    * For edge points, the closest edge point in the previous scan is found along with the closest edge point in the neighbouring scan lines. The point-to-line distance is then used as a residual.
    * For planar points, the closest planar point in the previous scan is found along with the closest planar points both on the same scan line and the neighbouring line. These three points are used to form a plane and then the point-to-plane distance is used as a residual.
* Once all the residuals have been calculated, an optimization to minimize the distances is performed. Bisquare weighting is applied to discount large correspondences.
* Transform the pointcloud using recovered motion and match against aggregated map. The same feature matching is applied, only this time the matching is between the local aggregate map and the undistorted pointcloud with more features being used.

### What's nice
* Motion distortion used as source of information
* Lightweight compared to ICP (at least the PCL version)
* Code is built in chunks; feature extraction, odometry, mapping
* Could try different features, perhaps incorporating intensity information

### Not so nice
* Performance in unstructured environments isn't so good, and may blow up.
* This is odometry, so no loop closure.

## Cartographer [Repo](https://github.com/googlecartographer)

Validator: Ben

License: Apache V2

### Technical summary ([paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf))

This package uses a combination of local scan-matching and spare pose graph optimisation. Has 2D and 3D versions. The map is a collection of submaps, each of which is a probability grid (flat or voxels).
All matchers are trying to maximise probability of current scan given the current probability grid. Upper bounds are calculated by inflating probability grids. The grids are pre-computed for each submap once it is finished.

Scan matchers:
* Fast_correlative_scan_matcher and real_time_correlative_scan_matcher: Implementations of [this](https://april.eecs.umich.edu/pdfs/olson2009icra.pdf). The fast version is faster but the map is immutable after construction. Has been extended to 3D
* There is also a version using ceres. The same probability formulation as above is used, but rather than branch-and-bound grid-search the problem is cast as a nonlinear least squares and passed to ceres.

Estimator:
* Quaternion-based UKF used to provide initial pose estimates. Based on [this] (https://pdfs.semanticscholar.org/3085/aa4779c04898685c1b2d50cdafa98b132d3f.pdf)

Pose-graph optimisation:
* Once a submap is finished (based on fixed number of scans) it is used with current scans: each scan is matched against nearby submaps, and if the match is good (based on likelihood estimate) it is used as a constraint for optimisation. The quantities being optimised are the global poses of the robot as well as the global position of the submaps given the relative poses of the robot within each submap as residuals.
* Implementation of [Sparse Pose Adjustment](http://www.willowgarage.com/sites/default/files/spa2d.pdf)

[Running on test track bag](https://www.youtube.com/watch?v=qdKBwwr5iH8)

### What's good
* Fairly modular, could probably bit and choose with some effort.
* Integrated ROS package is available.
* Provides a nice implementation for pose-graphs we might be able to take (or at least compare against)

### What's not so good
* Documentation for code is currently lacking, and since it is ~47k lines of code, trying to figure out what it is doing from the code can be time-consuming.
* Results from a bag at the test track are underwhelming, but may not be configured correctly.
* Doesn't seem to be built in a way that it can use previous maps.
