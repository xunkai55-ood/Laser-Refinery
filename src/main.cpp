#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "get_contour2.h"

using namespace std;

/* ros messages */
ros::Subscriber scan_raw;
ros::Publisher scan_refined;

/* extract contours parameters */
double adjacentAcceptDistance  = 0.03;
double maxJoinDistance         = 15;
int    maxSkipPoints           = 4;
int    endTrimPoints           = 3;
double startContourMaxDistance = 0.2;
double maxDistanceRatio        = 5;
double alwaysAcceptDistance    = 0.05;
double minPointsPerContour     = 20;

/* contour extractor */
ContourExtractor refinery;

void scanCallback(const sensor_msgs::LaserScan msg)
{
    /* publish contour msg */
    refinery.compute_contours(msg);
    scan_refined.publish(refinery.get_contour_msg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_refinery");
    ros::NodeHandle nh;

    scan_raw = nh.subscribe<sensor_msgs::LaserScan>("scan", 100, scanCallback);
    scan_refined = nh.advertise<sensor_msgs::LaserScan>("scan2", 100);

    refinery.set_params(maxSkipPoints, endTrimPoints, adjacentAcceptDistance, maxJoinDistance, startContourMaxDistance, minPointsPerContour, maxDistanceRatio, alwaysAcceptDistance);

    ros::spin();
}