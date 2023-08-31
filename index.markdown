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
  - name: Jonathon Michaux
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
    text: Arxiv HP
    url: https://arxiv.org/
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

# Overview Video (add link to youtube video)
<div class="fullwidth">
<video controls="" style="background-color:black;width:100%;height:auto;aspect-ratio:16/9;"></video>
</div>

# Abstract

A key challenge to ensuring the rapid transition of robotic systems from the industrial sector to more ubiquitous applications is the development of algorithms that can guarantee safe operation while in close proximity to humans.
Motion planning and control methods, for instance, must be able to certify safety while operating in real-time in arbitrary environments and in the presence of model uncertainty. 
This paper proposes Wrench Analysis for Inertial Transport using Reachability (WAITR), a certifiably safe motion planning and control framework for serial link manipulators that manipulate unsecured objects in arbitrary environments. 
WAITR uses reachability analysis to construct over-approximations of the contact wrench applied to unsecured objects, which captures uncertainty in the manipulator dynamics, the object dynamics, and contact parameters such as the coefficient of friction. 
An optimization problem formulation is presented that can be solved in real-time to generate provably-safe motions for manipulating the unsecured objects. 
This paper illustrates that WAITR outperforms state of the art methods in a variety of simulation experiments and demonstrates its performance in the real-world.

---

# [Method](#method)
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

Example of a failure due to contact constrainsts being turned off:
<div class="fullwidth">
<video controls="" width="100%">
    <source src="fetch_tray_fail.mkv">
</video>
</div>

Example of a success:
<div class="fullwidth">
<video controls="" width="100%">
    <source src="fetch_tray_fail.mkv">
</video>
</div>
  
</div>

---

# Hardware Results
<div markdown="1" class="content-block grey justify no-pre">

Video first shows an example of a failure due to contact constrainsts being turned off, then it shows a success when contact constraints are turned on. Both scenarios use the same global start and goal, as well as the same naive high level planner.
<div class="fullwidth">
<video controls="" width="100%">
    <source src="force_closure_720.mp4">
</video>
</div>

</div>

---
{::comment}
$$
\begin{align*}
  & \phi(x,y) = \phi \left(\sum_{i=1}^n x_ie_i, \sum_{j=1}^n y_je_j \right)
  = \sum_{i=1}^n \sum_{j=1}^n x_i y_j \phi(e_i, e_j) = \\
  & (x_1, \ldots, x_n) \left( \begin{array}{ccc}
      \phi(e_1, e_1) & \cdots & \phi(e_1, e_n) \\
      \vdots & \ddots & \vdots \\
      \phi(e_n, e_1) & \cdots & \phi(e_n, e_n)
    \end{array} \right)
  \left( \begin{array}{c}
      y_1 \\
      \vdots \\
      y_n
    \end{array} \right)
\end{align*}
$$

<div markdown="1" class="content-block grey justify">
# Citation

*Insert whatever message*

```bibtex
@article{nash51,
  author  = "Nash, John",
  title   = "Non-cooperative Games",
  journal = "Annals of Mathematics",
  year    = 1951,
  volume  = "54",
  number  = "2",
  pages   = "286--295"
}
```
</div>
{:/comment}
