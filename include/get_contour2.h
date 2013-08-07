#ifndef LASER_REFINERY_GET_CONTOUR2_H_
#define LASER_REFINERY_GET_CONTOUR2_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <cmath>

#define PI 3.1415926
#define eps 1e-8
#define MAXN 1200
#define MAX_VALUE  100000
#define CONT_FAR 1000000

#define sqr(x) ((x)*(x))

inline double taylorCos(double x)
{
	return 1 - x * x / 2 + x * x * x * x /24;
}

inline double quickCos(double x)
{
	if (x < PI / 2) 
		return taylorCos(x);
	else if (x < PI)
		return -taylorCos(PI - x);
	else if (x < PI * 1.5)
		return -taylorCos(x);
	else
		return cos(x);
}

class ContourExtractor
{
private:
    bool flag;
	// params
	int maxSkipPoints; //number of next points count in finding joins, must be positive
    int endTrimPoints; // number of trimmed points at each end of contour
	double adjacentAcceptDistance; 
	double sqAdjacentAcceptDistance;//always accept, set the dis = -1
	double maxJoinDistance;
	double sqMaxJoinDistance; //max dis between joins accept
	double startContourMaxDistance;
	double sqStartContourMaxDistance; //startContourMaxDistance * maxDistanceRatio is the distance to start a new contour
	double maxDistanceRatio;
	double sqMaxDistanceRatio; //the ratio of distance jump between joins, when start a new contour
	double alwaysAcceptDistance;
	double sqAlwaysAcceptDistance; // if the john.distance < alwaysAcceptDistance, always accept the joint
	int minPointsPerContour; //min points number in contours
	//
	sensor_msgs::LaserScan msg;
	//base_dis*range = default dis between adjacent scan points
	float base_dis;
    double sqBaseDis;
    int right[MAXN];
    int contours[MAXN]; 
    int cont_idx[MAXN];
    int n_cont_idx, m_conts;

	inline double sqrDistance(int i, int j);

public:
	ContourExtractor();
	void set_params(int _maxSkipPoints, int _contTrim, double _adjacentAcceptDistance, double _maxJoinDistance, double _startContourMaxDistance, int _minPointsPerContour, double _maxDistanceRatio, double _alwaysAcceptDistance);
	void compute_contours(sensor_msgs::LaserScan _msg);
	sensor_msgs::LaserScan get_contour_msg(int reverse = 0);

};

#endif
