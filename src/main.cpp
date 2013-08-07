#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "laser_scan_matcher/get_contour2.h"
#include "laser_scan_matcher/get_contour.h"

using namespace std;

/* ros messages */
ros::Subscriber scan_sub;
ros::Publisher scan_pub1, scan_pub2;

/* extract contours parameters */
double adjacentAcceptDistance  = 0.03;
double maxJoinDistance         = 15;
double maxSkipPoints           = 4;
double startContourMaxDistance = 0.2;
double maxDistanceRatio        = 5;
double alwaysAcceptDistance    = 0.05;
double minPointsPerContour     = 20;

/* contour extractor */
contour_extractor contour_handler;
contour_extractor_od contour_handler_od;

int callbackTime = 0;
void scanCallback(const sensor_msgs::LaserScan msg)
{
    /* save as txt */
    callbackTime++;
    char fn[100] = "";
    sprintf(fn, "/home/badpoet/bag2txt/square_raw/scan_%06d", callbackTime);
    FILE* saveFile = fopen(fn, "w");

    int n = msg.ranges.size();
    for (int i = 0; i < n; i++)
        fprintf(saveFile, "%.6f\n", msg.ranges[i]);
    fclose(saveFile);

    /* publish contour msg */
    contour_handler_od.cmpt_contours(msg);
    scan_pub1.publish(contour_handler_od.get_contour_msg());
    contour_handler.cmpt_contours(msg);
    sensor_msgs::LaserScan msg2 = contour_handler.get_contour_msg();
    scan_pub2.publish(msg2);

    sprintf(fn, "/home/badpoet/bag2txt/data1/scan_%06d", callbackTime);
    saveFile = fopen(fn, "w");

    n = msg2.ranges.size();
    for (int i = 0; i < n; i++)
        fprintf(saveFile, "%.6f\n", msg2.ranges[i]);
    fclose(saveFile);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_handler");
    ros::NodeHandle nh;

    scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 100, scanCallback);
    scan_pub1 = nh.advertise<sensor_msgs::LaserScan>("scan1", 100);
    scan_pub2 = nh.advertise<sensor_msgs::LaserScan>("scan2", 100);

    contour_handler.set_params(maxSkipPoints,adjacentAcceptDistance,maxJoinDistance,
    startContourMaxDistance,minPointsPerContour,maxDistanceRatio,alwaysAcceptDistance);
    //contour_handler2.set_params(maxSkipPoints,adjacentAcceptDistance,maxJoinDistance,
    //startContourMaxDistance,minPointsPerContour,maxDistanceRatio,alwaysAcceptDistance);

    ros::spin();
}