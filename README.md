Laser refinery for ROS
===================================

Overview
-----------------------------------

A package that refines the laser data before you use them. Node `laser refinery` will delete some bad points. The alpha version contributed a lot in THRONE's program in IARC'2013.

Installing
-----------------------------------

### From source ###

Assumed that you're familiar with git.

Download the stack from our repository:

    git clone https://github.com/badpoet/Laser-Refinery

Install any dependencies using [[rosdep]].

    rosdep install laser_refinery

Compile the stack:

    rosmake laser_refinery

Run
-----------------------------------

Run the node:

	rosrun laser_refinery main

The default raw laser topic is `scan` while the default refined laser topic is `scan2`.

More info
-----------------------------------

ToDo
 
