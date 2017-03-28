# Visual Features

### Viewpoint Invariance
* Needed for Relocalization, loop closing, multi-agent SLAM. Descriptors look the same upto 30-35 degrees.
* Use multiple descriptors from different views and retain only most informative descriptors (PCA, KPCA, angle>25 etc)
* Use of range sensor or/and IMU to estimate viewpoints
* Active Vision: Position camera in different positions
* A single descriptor to characterize multiple descriptors. Could use branch and bound technique to find most similar descriptor.
* Salient features to better estimate viewpoint
* Locate features based on semantic understanding of objects. Grouping of features
* Storing multiple features on objects. Allow certain parameters to vary. Match only on stable features (Lines etc.)
