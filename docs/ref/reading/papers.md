# Great SLAM Papers

## SLAM Surveys
* 2006 Famous Hugh Durrant-Whyte Survey, claiming SLAM is solved [[Part I - State of the art][DurrantBailey2006]], [[Part II- Recursive Bayesian Formulation and open issues][BaileyDurrant2006]]
    * Excellent summary of the filter and graph approaches with point features and Bayesian inference over Gaussians
    * Part I Abstract: This paper describes the simultaneous localization and mapping (SLAM) problem and the essential methods for solving the SLAM problem and summarizes key implementations and demonstrations of the method. While there are still many practical issues to overcome, especially in more complex outdoor environments, the general SLAM method is now a well understood and established part of robotics. Another part of the tutorial summarized more recent works in addressing some of the remaining issues in SLAM, including computation, feature representation, and data association.
    * Part II Abstract: This paper discusses the recursive Bayesian formulation of the simultaneous localization and mapping (SLAM) problem in which probability distributions or estimates of absolute or relative locations of landmarks and vehicle pose are obtained. The paper focuses on three key areas: computational complexity; data association; and environment representation.
* 2016 Young guys + Scaramuzza, Neira, Reid and Leonard [[SLAM Survey Paper][CadenaEtAl2016]]
    * Excellent survey of SLAM today, updating the hard parts that remain.
    * Nice summary of MAP Factor Graph SLAM
    * Defines three epochs: Classical SLAM (up to 2006), Algorithmic Analysis (up to today), Robustification (going forward)
    * Do robots need SLAM? yes, Is SLAM solved? Basic yes, but not for in demanding environments or for demanding applications
    * Major open areas:
        * Robustness: backend recovery, hw failure recovery, relocalization, deformable, time-varying environment
        * Scalability: sparsification, parallelization, multi-robot, learning-forgetting, resource constraint aware
        * Metric Reasoning (map representations): solved in 2D, harder in 3D.  Progression of map representations - features, raw-dense, direct methods, boundaries (edges), surfaces, planes, object based, semantic.  High level representations that remain expressive, optimal representations and automatic adaptation of representations triggered by environment, task.
        * Semantic reasoning: Identify spaces, objects and relationships between them.  Not just classification, much more needed to properly define tasks.
        * Theoretical Tools: convergence guarantees, failure prediction with spurious measurements, convexity argument - solving primal and dual problems to provide an optimality bound, outlier resiliency theory.
        * Active SLAM: Needs fast, accurate measurement predictions, exploration/exploitation trade off not clear, when is it necessary/useful? Performance guarantees through active perception.
        * New sensors: can lead to different paradigms, such as event triggered cameras.
* 2015 Arthur Huletski, Dmitriy Kartashov, Kirill Krinkin [[Evaluation of the Modern Visual SLAM Methods][HuletskiEtAl2015]]
    This paper evaluates 2013-mid to 2015 ORB, LSD, L, OpenRat visual SLAM algorithms on the TUM RGB-D benchmark. One of the important questions they mention is: What assumptions and approximations were made while creating the SLAM algorithm ? This will help us know our limitations.
    Metrics that decide whether a SLAM algorithm is suitable for a particular task:
    * Localization accuracy (RMSE between predicted positions and ground truth)
    * CPU usage for computational effectiveness
    * Peak memory consumption allows to estimate memory requirements. Important for limited resources
    * Camera FPS
    * Map quality (occupancy grid outputs a different map from ground truth)
    * Robustness [results over multiple runs] and Convergence [time necessary for Minimization of localization error]
    Conclusion: While the algorithms above can show good performance, they don’t show stable tracking and low error. Robust results aren’t available out of the box. Parameters need to be tuned.
    * RatSLAM needs the video to be slowed down by a factor of 2.
    * ORB SLAM and LSD SLAM require tweaking of parameters.
    * FOV can also play an important factor.
    Drawbacks:
    * Method that estimates camera transformation should have at least medium level on noise (RatSLAM draw­ back);
    * An algorithm should be scale-aware (ORB-SLAM drawback);
    * In case of tracking loss an algorithm should use some fallback strategy instead of giving up (LSD-SLAM drawback).

## Seminal works - a partial list

* [[1987 - The original EKF SLAM from Cheeseman, Smith and Self][CheesemanEtAl87]]: the original, a beautiful paper formulating SLAM using Gaussian uncertainty for the first time.
* [1997 - Lu and Milios Scan Registration Graph SLAM][LuMilios97]: York University shows up big with 2D lidar scan registration in hallways, includes ICP and graphSLAM foundations.
* [[2002 - Rao-Blackwellized Particle SLAM][MontemerloEtAl2002]]: Thrun and Montemerlo show how particle filters can handle extreme nonlinearities, but run into a lot of problems in implementation.
* [[2003 - MonoSLAM][]]: The first real monocular camera only SLAM paper, courtesy of Andy Davison.
* [[2007 - Parallel Tracking and Mapping][KleinMurray2007]]: The case for separate mapping and localization threads, shows how maps don't need to be simultaneously updated at all.
* [[2010 - Why Filter?][StrasdatEtAl2010]] The case for bundle adjustments over continuous filtering, allowing the optimization to go back and correct past linearization mistakes.
* [[2011 - KinectFusion][NewcombeEtAl2011]]  The first dense rgbd SLAM, showing us a path to merging of the offline reconstruction methods with online SLAM (same problem different timescales).
* [[2014 - SVO:Fast Semi-direct Monocular Visual Odometry][ForsterEtAl2014]] A combination of direct and feature based SLAM, claiming to get the best of both worlds.
* [[2014 - Large-Scale Direct Monocular SLAM][EngelEtAl2014]] Impressive jump in scale for direct methods that work on pixel intensities.
* [[2015 - ORB-SLAM][MurArtalEtAl15]] Solid feature based implementation of visual slam, extended to omni and stereo cameras in ORB-SLAM2.
* [[2016 - Direct Sparse Odometry][EngelEtAl2016]] Direct method that does not work on all high gradient pixels but a sparse subset of them. Fast, and accurate. Similar in concept to semi-direct method above.


## Our work

* NASA Sample Return Robot Challenge: [[NASA paper][DasEtAl2014]]
* Multi-camera parallel tracking and mapping: [[MCPTAM Main][TribouEtAl2015]] and [[Degeneracy Analysis][TribouEtAl2016]]
* Dynamic camera cluster calibration : [[IROS2016][DasWaslander2016]] and [[IROS2017][RebelloEtAl2017]]


## Courses and other online resources

* [[Visual SLAM List][VSLAMList]]

## References

- [Durrant-Whyte, H., & Bailey, T. (2006). Simultaneous localization and mapping: part I. IEEE robotics & automation magazine, 13(2), 99-110.][DurrantBailey2006]

- [Bailey, T., & Durrant-Whyte, H. (2006). Simultaneous localization and mapping (SLAM): Part II. IEEE Robotics & Automation Magazine, 13(3), 108-117.][BaileyDurrant2006]

- [Cadena, C., Carlone, L., Carrillo, H., Latif, Y., Scaramuzza, D., Neira, J., & Leonard, J. J.  (2016). Simultaneous Localization And Mapping: Present, Future, and the Robust-Perception Age.  arXiv preprint   arXiv:1606.05830.][CadenaEtAl2016]

- [Cheeseman, P., Smith, R., & Self, M. (1987). A stochastic map for uncertain spatial relationships. In 4th International Symposium on Robotic Research (pp. 467-474).][CheesemanEtAl87]

- [F. Lu and E. Milios. Globally consistent range scan alignment for environment mapping. Autonomous Robots (AR), 4(4):333–349, 1997][LuMilios97]

- [Montemerlo, M., Thrun, S., Koller, D., and Wegbreit, B. (2002) FastSLAM: A factored solution to the simultaneous localization and mapping problem. Proceedings of the AAAI National Conference on Artificial Intelligence, Edmonton, Canada, pp.593–598.][MontemerloEtAl2002]

- [Davison, A. J., Reid, I. D., Molton, N. D., & Stasse, O. (2007). MonoSLAM: Real-time single camera SLAM. IEEE transactions on pattern analysis and machine intelligence, 29(6).][DavisonEtAl2007]

- [Klein, G., & Murray, D. (2007, November). Parallel tracking and mapping for  small AR workspaces.  In Mixed and Augmented Reality, 2007. ISMAR 2007. 6th IEEE and ACM International Symposium on (pp. 225-234). IEEE.][KleinMurray2007]

- [Strasdat, H., Montiel, J. M. M., & Davison, A. J. (2010, May). Real-time monocular SLAM: Why filter?. In Robotics and Automation (ICRA), 2010 IEEE International Conference on (pp. 2657-2664). IEEE.][StrasdatEtAl2010]

- [Strasdat, H., Montiel, J. M., & Davison, A. J. (2012). Visual SLAM: why filter?. Image and Vision Computing, 30(2), 65-77.][StrasdatEtAl2012]

- [Newcombe, R. A., Izadi, S., Hilliges, O., Molyneaux, D., Kim, D., Davison, A. J., ... & Fitzgibbon, A. (2011, October). KinectFusion: Real-time dense surface mapping and tracking. In Mixed and augmented reality (ISMAR), 2011 10th IEEE international symposium on (pp. 127-136). IEEE.][NewcombeEtAl2011]

- [Engel, J., Schöps, T., & Cremers, D. (2014, September). LSD-SLAM: Large-scale direct monocular SLAM. In European Conference on Computer Vision (pp. 834-849). Springer International Publishing.][EngelEtAl2014]

- [Leutenegger, S., Lynen, S., Bosse, M., Siegwart, R., & Furgale, P. (2015). Keyframe-based visual–inertial odometry using nonlinear optimization. The International Journal of Robotics Research, 34(3), 314-334.][LuteneggerEtAl2015]

- [Nardi, L., Bodin, B., Saeedi, S., Vespa, E., Davison, A. J., & Kelly, P. H. (2017). Algorithmic Performance-Accuracy Trade-off in 3D Vision Applications Using HyperMapper. arXiv preprint arXiv:1702.00505.][NardiEtAl2017]

- [Das, A., Diu, M., Mathew, N., Scharfenberger, C., Servos, J., Wong, A., ...  & Waslander, S. L. (2014). Mapping, planning, and sample detection strategies for autonomous exploration. Journal of Field Robotics, 31(1), 75-106.][DasEtAl2014]

- [Tribou, M. J., et al. (2015). Multi-camera parallel tracking and mapping with non-overlapping fields of view. The International Journal of Robotics Research 34.12:1480-1500][TribouEtAl2015]

- [Tribou, M. J., Wang, D. W., & Waslander, S. L. (2016). Degenerate motions in multicamera cluster SLAM with non-overlapping fields of view. Image and Vision Computing, 50, 27-41.][TribouEtAl2016]

- [tzutalin, "The list of vision-based SLAM / Visual Odometry open source projects, libraries, dataset, tools, and studies", Github, tzutalin/awesome-visual-slam.][VSLAMList]

- [Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. IEEE > Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, 2015.][MurArtalEtAl15]

- [Das, A., & Waslander, S. L. (2016, October). Calibration of a dynamic camera cluster for multi-camera visual SLAM. In Intelligent Robots and Systems (IROS), 2016 IEEE/RSJ International Conference on (pp. 4637-4642). IEEE.][DasWaslander2016]


[CheesemanEtAl87]: https://pdfs.semanticscholar.org/76a6/c5352a0fbc3fec5395f1501b58bd6566d214.pdf
[LuMilios97]: http://www-robotics.usc.edu/~gaurav/CS547/milios_map.pdf
[MontemerloEtAl2002]: http://www.aaai.org/Papers/AAAI/2002/AAAI02-089.pdf
[DurrantBailey2006]: https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf
[BaileyDurrant2006]: http://www-personal.acfr.usyd.edu.au/tbailey/papers/slamtute2.pdf
[DavisonEtAl2007]: https://spiral.imperial.ac.uk/bitstream/10044/1/695/1/MonoSLAM%20Real-Time%20Single%20Camera%20SLAM.pdf
[KleinMurray2007]: http://www.robots.ox.ac.uk/~gk/publications/KleinMurray2007ISMAR.pdf
[GrisettiEtAl2010]: http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf
[StrasdatEtAl2010]: https://www.doc.ic.ac.uk/~ajd/Publications/strasdat_etal_icra2010.pdf
[NewcombeEtAl2011]: https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf
[StrasdatEtAl2012]: https://www.doc.ic.ac.uk/~ajd/Publications/strasdat_etal_ivc2012.pdf
[EngelEtAl2014]: https://vision.in.tum.de/_media/spezial/bib/engel14eccv.pdf
[HuletskiEtAl2015]: http://ieeexplore.ieee.org/document/7382963/
[LuteneggerEtAl2015]: http://www.roboticsproceedings.org/rss09/p37.pdf
[CadenaEtAl2016]: http://www.arxiv.org/pdf/1606.05830v2.pdf
[NardiEtAl2017]: https://arxiv.org/pdf/1702.00505.pdf
[DasEtAl2014]: http://onlinelibrary.wiley.com/doi/10.1002/rob.21490/full
[TribouEtAl2015]: http://journals.sagepub.com/doi/abs/10.1177/0278364915571429
[TribouEtAl2016]: http://www.sciencedirect.com/science/article/pii/S0262885616300038
[MurArtalEtAl15]: http://webdiis.unizar.es/~raulmur/orbslam/
[DasWaslander2016]: http://ieeexplore.ieee.org/abstract/document/7759682/
[VSLAMList]: https://github.com/tzutalin/awesome-visual-slam
