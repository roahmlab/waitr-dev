---
# Front matter. This is where you specify a lot of page variables.
layout: default
title:  "WAITR"
date:   2023-06-16 03:03:01 -0400
description: >- # Supports markdown
  Wrench Analysis for Inertial Transport using Reachability
show-description: true

# Add page-specific mathjax functionality. Manage global setting in _config.yml
mathjax: true
# Automatically add permalinks to all headings
# https://github.com/allejo/jekyll-anchor-headings
autoanchor: false

authors:
  - name: Zachary Brei
    email: breizach@umich.edu
    # footnotes: 1
  - name: Jonathan Michaux
    # url: https://buildingatom.io
    email: jmichaux@umich.edu
    # footnotes: 2
  - name: Bohao Zhang
    email: jimzhang@umich.edu
    # footnotes: 2
  - name: Patrick Holmes
    # footnotes: 2
  - name: Ram Vasudevan
    email: ramv@umich.edu
    # footnotes: 2

author-footnotes:
  All authors affiliated with the department of Mechanical Engineering and Department of Robotics of the University of Michigan, Ann Arbor.
  
  
links:
  - icon: arxiv
    icon-library: simpleicons
    text: Arxiv
    url: https://arxiv.org/abs/2309.03111
  - icon: github
    icon-library: simpleicons
    text: Code
    url: https://github.com/roahmlab/waitr-dev
  - icon: bi-file-earmark-text
    icon-library: bootstrap-icons
    text: Supplementary Appendices
    url: RAL_WAITR_Appendices.pdf
    

# End Front Matter
---

{% include sections/authors %}
{% include sections/links %}

---

# Overview Video
<div class="fullwidth">
<iframe style="aspect-ratio: 16/9; height: 100%; width: 100%;" src="https://www.youtube.com/embed/-n6SwmylyX4" title="Serving Time: Real-Time, Safe Motion Planning and Control for Manipulation of Unsecured Objects" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>

# Abstract

A key challenge to ensuring the rapid transition of robotic systems from the industrial sector to more ubiquitous applications is the development of algorithms that can guarantee safe operation while in close proximity to humans.
Motion planning and control methods, for instance, must be able to certify safety while operating in real-time in arbitrary environments and in the presence of model uncertainty. 
This paper proposes Wrench Analysis for Inertial Transport using Reachability (WAITR), a certifiably safe motion planning and control framework for serial link manipulators that manipulate unsecured objects in arbitrary environments. 
WAITR uses reachability analysis to construct over-approximations of the contact wrench applied to unsecured objects, which captures uncertainty in the manipulator dynamics, the object dynamics, and contact parameters such as the coefficient of friction. 
An optimization problem formulation is presented that can be solved in real-time to generate provably-safe motions for manipulating the unsecured objects. 
This paper illustrates that WAITR outperforms state of the art methods in a variety of simulation experiments and demonstrates its performance in the real-world.

---

# Method
<div markdown="1" class="content-block grey justify no-pre">

![Robot motions, joint torques and contact wrenches are overapproximated using polynomial zonotopes.](MethodFigure_v5_column_small-01.jpg)
  
This paper considers the problem of safe motion planning for manipulation of unsecured objects with uncertain dynamics such as manipulating an unsecured cup filled with an uncertain mass around randomly placed obstacles (red) such that the cup does not move relative to the tray supporting it. 
WAITR operates in receding-horizon fashion, moving from a start configuration (blue) to a global goal (green) by repeatedly generating new motion plans in real-time. 
In each motion planning iteration, WAITR calculates a reachable set (blue and purple) for the contact wrench between the manipulator and the object as well as a Forward Reachable Set (FRS) for the whole manipulator system for a continuum of possible motion plans. 
The FRS is shown in purple in a) for a single planning iteration. 
WAITR solves a constrained trajectory optimization problem to find a collision-free motion in this FRS that does not result in relative motion while making progress towards an intermediate waypoint (grey) and the global goal. 
Parts c)-e) show the contact constraints enforced during a hardware experiment for a single planning iteration.

</div>

---

# Simulation Results
<div markdown="1" class="content-block grey justify no-pre">

This video is a simulation trial performed to test WAITR against ARMOUR and is one of the results reported in Tab. 2 of the paper. 
<div class="fullwidth">
<iframe style="aspect-ratio: 16/9; height: 100%; width: 100%;" src="https://www.youtube.com/embed/FHla5Lob5WU?si=3Z9s9nVwl274MF7u" title="WAITR Simulation Trial" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>

This video is another scenario where a fetch robot is used to manipulate a block. A pybullet environment is used, which allows the block to be dropped into place on the tray and have the pybullet physics engine simulate the interaction between the block and tray throughout the trajectory. The 
Example of a failure due to contact constrainsts being turned off:
<div class="fullwidth">
<iframe style="aspect-ratio: 16/9; height: 100%; width: 100%;" src="https://www.youtube.com/embed/8r16CsglxLg?si=kxZf8YNiGaLTg9rC" title="WAITR Simulation Trial" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>
</div>

---

# Hardware Results
<div markdown="1" class="content-block grey justify no-pre">

The overview video above shows the hardware experiments that are referenced in the paper. 

This video is another scenario where the robot uses a straight-line high level planner which selects waypoints along a straight line in joint space between the current configuration and the goal. The first part shows an example of a failure due to contact constrainsts being turned off, then it shows a success when contact constraints are turned on. Both scenarios use the same global start and goal, as well as the same straight line high level planner.
<div class="fullwidth">
<iframe style="aspect-ratio: 16/9; height: 100%; width: 100%;" src="https://www.youtube.com/embed/_JzhxYgtu0w?si=laS-QZk0Ha4dhOBz" title="WAITR Hardware Comparison with ARMOUR" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>



</div>

---
