# Introduction

Factor graphs are a graphical modelling method which excel in optimizing estimation problems such as SLAM [[1][Dellaert2012]]. This document serves as an introduction; please refer to the references below for more thorough explanations and examples.

# Preliminaries

## Graphical Modelling and Representation

A graphical model is a probabilistic model which illustrates the conditional dependence between random variables. A factor graph is a model that employs a *bipartite* graph; a graph consisting of two distinct nodes.The two nodes in a factor graph are *variables* and *factors*, which are connected together by *edges* [[2][Eckford2010]]. Illustrated below is an example factor graph, with 3 variables X1, X2, X3, and four factors f1, f2, f3, and f4 [[3][Wiki]]. As a convention, variables are usually denoted by circles [[2][Eckford2010]].

<img src="images/factor_graphs/factorgraph1.jpg" width="350px" alt="Example Factor Graph" />

The edges illustrate the relationship between the factors and the variables. If the variable appears in the factor, we can connect an edge between the two. Since the edges illustrate the associated variables, we can express the factors as such: **f1(X1), f2(X1, X2), f3(X1, X2), f4(X2, X3)**.

## Factors

Factors are probability statements illustrating the conditional dependencies between the variables. These are usually derived from measurements or mathematical fundamentals [[1][Dellaert2012]]. As we know from probability theory, any joint probability model with any number of variables can be broken down into factors [[2][Eckford2010]].

For example, we can state that the probability of two variables, *a*, and *b*, can be expressed as: **f(a, b) = f(a | b)\*f(b)**; a factor of the two variables. This example can also be extended to our above example - if we know that the probability of *X1*, *X2*, and *X3*, is expressed as **f(X1, X2, X3) = f1(X1)\*f2(X1, X2)\*f3(X1, X2)\*f4(X2,X3)**, we can use this to derive the above image.

# Applications

Now that we have discussed the preliminaries and conventions, we can look at an example.

## Robot Localization

One application is robot localization, as discussed by Dellaert [[1][Dellaert2012]]. Let us imagine we have a robot from which we can obtain odometry and GPS measurements. The variables *x1, x2*, and *x3* denote 3 poses of the robot over time while *z1, z2*, and *z3* denote the measurement of that pose. The factor graph can be derived as follows:

<img src="images/factor_graphs/factorgraph2.png" width="400px" alt="Example Factor Graph 2" />

The GPS measurements are illustrated by **f1(x1; z1), f2(x2; z2)**, and **f3(x3; z3)**. As measurements only depend on the one state they are measuring, these are unary factors. The odometry measurements are shown as the dots between the variables, as this information provides the relation between the two poses. This can be described as **f4(x1, x2; o1)**, and **f5(x2, x3; o2)**, where *o* refers to the odometry measurement at that point. As this depends on two variables, this is a binary factor. In addition to this, each of these measurement factors will have associated noise.

### Optimization and Solving

Now that the factor graph is set up, we can now find an optimal solution for the variables, given the measurement factors, and the actual measured values. GTSAM looks to perform a maximum a-posteriori (MAP) inference, which finds the values of the variables that **maximizes** the product of the factors [[1][Dellaert2012]].

## SLAM Applications
As mentioned by Dellaert, factor graphs are especially well suited for SLAM applications, such as *poseSLAM, Landmark SLAM*, and *Visual SLAM* [[1][Dellaert2012]]. This is the case because the factors are generally only associated with a few of the many variables associated with the problem. For example, the localization problem above only had 3 pose variables, but this increases constantly as the robot continues to move over time. However, although there are more variables, each odometry and GPS factor is still only binary or unary, respectively. This structure in combination with breakthroughs in solving sparse linear alegbra equations has led to computationally efficient, accurate SLAM algorithms [[5][Grisetti2010]].


# References and Resources
1.  [Dellaert, F. (2012). Factor Graphs and GTSAM: A Hands-on Introduction][Dellaert2012]
2.  [Eckford, A. (2010). Factor Graphs and the Sum-Product Algorithm, *Video Series*][Eckford2010]
3.  [Wikipedia. Factor Graph][Wiki]
4.  [Grisetti, G., Kümmerle, R., Stachniss, C., Burgard, W. (2010). A Tutorial on Graph-Based SLAM][Grisetti2010]
5.  [Koller, D (2012). Probabilistic Graphical Models, *Video Series*][Koller2012]

[Dellaert2012]: https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
[Eckford2010]: https://www.youtube.com/watch?v=8H5LJVgtzsg
[Wiki]: https://en.wikipedia.org/wiki/Factor_graph
[Grisetti2010]: http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf
[Koller2012]: https://www.youtube.com/watch?v=6ODl1rxoT14&list=PL50E6E80E8525B59C&index=5

