# SLAM Benchmarking and Evaluation {#vision_benchmarking}

## A Framework for Evaluating Visual SLAM [paper](http://www.bmva.org/bmvc/2009/Papers/Paper396/Paper396.pdf) (http://vslam.inf.tu-dresden.de/)

Their main point is that there is no performance measures for visual SLAM. Recorded sequences lack ground truth and simulated environments aren’t true reflections of what we might see in the real world. Hence by using rendered scenes, not only do we get ground truth but also can recreate real world scenes. High-quality rendered images are close enough to real-world images to violate many of the assumptions usually made in the image processing part of VSLAM. Some papers give quantitative results on restricted scene types. Framework consists of four components: a repository, a rendering system, an interface for VSLAM systems and an evaluation system. Able to generate motion blur.
