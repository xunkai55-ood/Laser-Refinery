Laser refinery for ROS
===================================

Overview
-----------------------------------

A package to refine laser data. It will subscribe your raw laser message and publish a refined one, without bad points. The refined data will be better used by other nodes like laser-scan-matcher.

Installing
-----------------------------------

### From source ###

Assumed that you're familiar with git.
Download the package from our repository:

	git clone https://github.com/badpoet/Laser-Refinery

Install any dependencies using [[rosdep]].

    rosdep install laser_refinery

Compile the package:

    rosmake laser_refinery

More info
-----------------------------------

(To be done)

 
