Laser-Refinery
==============

Overview
-----------------------------------

Laser scan refinery. The package contains can be used for cutting some bad points in laser scan data, to avoid potential accidents when you processing them.

Installing
-----------------------------------

We assume that you're familiar with git.
Download the package from our repository:

    git clone https://github.com/badpoet/Laser-Refinery.git

Install any dependencies using [[rosdep]].

    rosdep install laser_refinery

Compile the stack:

    rosmake laser_refinery

More info
-----------------------------------

Only effective when laser data is polar sorted.

The package use a fast linear algorithm.

The algorithm is only test on ROS-fuerte.

The default parameters in `main.cpp` performed well when using a
R311-HOKUYO-LASER2 hokuyo laser sensor indoors.

This package was used by THRONE (The team of Tsinghua Univ. in IARC 2013),
who was the only team completed IARC Mission 6.
