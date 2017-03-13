## Rotations, Lie Groups, Manifold operators
 * Good reference on Lie Algebra for SO(3) and SE(3), goes into a lot of nice detail for derivatives and such.  [Eathan Eade Link](http://www.ethaneade.org/lie.pdf), [ Ivancevic Link] (https://arxiv.org/pdf/1104.1106.pdf)
 * Prof Daniel Cremers course, videos on youtube (Initially, skim through lectures 3 and 4, though everything will be pretty useful as we progress).  [Link](https://www.youtube.com/watch?v=khLM8VV8LuM&list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4&index=3)
 * Box operators. [Link](https://pdfs.semanticscholar.org/4613/727ef686c6186cab69e6b8be8cb1fa3ba800.pdf)

## ICP and Scan Matching
  * Arun's Master's Thesis, which has descriptions and references for the major scan registration algorithms (ICP, G-ICP, NDT). [Link](https://uwspace.uwaterloo.ca/bitstream/handle/10012/7431/Das_Arun_2013.pdf?sequence=3)
  * The Point Cloud Library (PCL). [Link](http://www.pointclouds.org/documentation/tutorials/#registration-tutorial)

## Factor Graphs and MAP estimation
* Factor Graphs and Maximum-Liklihood (ML) estimation from GTSAM tutorial (ignore the code aspects for now, and focus on concepts). [Link](https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf)
* MAP from wikipedia [Link] (https://en.wikipedia.org/wiki/Maximum_likelihood_estimation)
* Putting ML estimation and Box operators together: SLAM on Manifolds, from the g2o documentation. (Again, ignore the code for now, and focus on the concepts). [Link] (https://github.com/RainerKuemmerle/g2o/blob/master/doc/g2o.pdf)

## SLAM Surveys
* 2006 Famous Hugh Durrant-Whyte Survey, claiming SLAM is solved [Part I - State of the art](http://ieeexplore.ieee.org/abstract/document/1638022/), [Part II- Recursive Bayesian Formulation and open issues] (http://ieeexplore.ieee.org/abstract/document/1678144/)
    * Excellent summary of the filter and graph approaches with point features and Bayesian inference over Gaussians
    * Part I Abstract: This paper describes the simultaneous localization and mapping (SLAM) problem and the essential methods for solving the SLAM problem and summarizes key implementations and demonstrations of the method. While there are still many practical issues to overcome, especially in more complex outdoor environments, the general SLAM method is now a well understood and established part of robotics. Another part of the tutorial summarized more recent works in addressing some of the remaining issues in SLAM, including computation, feature representation, and data association. 
    * Part II Abstract: This paper discusses the recursive Bayesian formulation of the simultaneous localization and mapping (SLAM) problem in which probability distributions or estimates of absolute or relative locations of landmarks and vehicle pose are obtained. The paper focuses on three key areas: computational complexity; data association; and environment representation.
* 2016 Young guys + Scaramuzza, Neira, Reid and Leonard [SLAM Survey Paper](https://arxiv.org/pdf/1606.05830.pdf)
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


# Uncategorized Slam Papers

- [Durrant-Whyte, H., & Bailey, T. (2006). Simultaneous localization and
  mapping: part I. IEEE robotics & automation magazine, 13(2),
  99-110.][DurrantBailey2006]

- [Bailey, T., & Durrant-Whyte, H. (2006). Simultaneous localization and
  mapping (SLAM): Part II. IEEE Robotics & Automation Magazine, 13(3),
  108-117.][BaileyDurrant2006]

- [Klein, G., & Murray, D. (2007, November). Parallel tracking and mapping for
  small AR workspaces.  In Mixed and Augmented Reality, 2007. ISMAR 2007. 6th
  IEEE and ACM International Symposium on (pp. 225-234).
  IEEE.][KleinMurray2007]

- [Grisetti, G., Kummerle, R., Stachniss, C., & Burgard, W. (2010). A tutorial
  on graph-based SLAM. IEEE Intelligent Transportation Systems Magazine, 2(4),
  31-43.][GrisettiEtAl2010]

- [Strasdat, H., Montiel, J. M. M., & Davison, A. J. (2010, May). Real-time
  monocular SLAM: Why filter?. In Robotics and Automation (ICRA), 2010 IEEE
  International Conference on (pp. 2657-2664). IEEE.][StrasdatEtAl2010]

- [Strasdat, H., Montiel, J. M., & Davison, A. J. (2012). Visual SLAM: why
  filter?. Image and Vision Computing, 30(2), 65-77.][StrasdatEtAl2012]

- [Engel, J., Schöps, T., & Cremers, D. (2014, September). LSD-SLAM:
  Large-scale direct monocular SLAM. In European Conference on Computer Vision
  (pp. 834-849). Springer International Publishing.][EngelEtAl2014]

- [Leutenegger, S., Lynen, S., Bosse, M., Siegwart, R., & Furgale, P. (2015).
  Keyframe-based visual–inertial odometry using nonlinear optimization. The
  International Journal of Robotics Research, 34(3),
  314-334.][LuteneggerEtAl2015]

- [Cadena, C., Carlone, L., Carrillo, H., Latif, Y., Scaramuzza, D., Neira, J.,
  ... & Leonard, J. J.  (2016). Simultaneous Localization And Mapping: Present,
  Future, and the Robust-Perception Age.  arXiv preprint
  arXiv:1606.05830.][CadenaEtAl2016]

- [Nardi, L., Bodin, B., Saeedi, S., Vespa, E., Davison, A. J., & Kelly, P. H.
  (2017). Algorithmic Performance-Accuracy Trade-off in 3D Vision Applications
  Using HyperMapper. arXiv preprint arXiv:1702.00505.
  Chicago][NardiEtAl2017]


[DurrantBailey2006]: https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf
[BaileyDurrant2006]: http://www-personal.acfr.usyd.edu.au/tbailey/papers/slamtute2.pdf
[KleinMurray2007]: http://www.robots.ox.ac.uk/~gk/publications/KleinMurray2007ISMAR.pdf
[GrisettiEtAl2010]: http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf
[StrasdatEtAl2010]: https://www.doc.ic.ac.uk/~ajd/Publications/strasdat_etal_icra2010.pdf
[StrasdatEtAl2012]: https://www.doc.ic.ac.uk/~ajd/Publications/strasdat_etal_ivc2012.pdf
[EngelEtAl2014]: https://vision.in.tum.de/_media/spezial/bib/engel14eccv.pdf
[LuteneggerEtAl2015]: http://www.roboticsproceedings.org/rss09/p37.pdf
[CadenaEtAl2016]: http://www.arxiv.org/pdf/1606.05830v2.pdf
[NardiEtAl2017]: https://arxiv.org/pdf/1702.00505.pdf
