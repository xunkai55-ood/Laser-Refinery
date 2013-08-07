#ifndef GET_CONTOUR2_H
#define GET_CONTOUR2_H

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <vector>
#include <iostream>
#include <sys/time.h>

#define PI 3.1415926
#define eps 1e-8
#define MAXN 1200
#define MAX_VALUE  100000
#define DEBUG_OUTPUT 1

#define CONT_TRIM 3
#define CONT_FAR 1000000

#define sqr(x) ((x)*(x))

using namespace std;

struct timeval tv;

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

class contour_extractor
{
private:
    bool flag;
	// params
	int maxSkipPoints; //number of next points count in finding joins, must be positive
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

	inline double sqrDistance(int i, int j)
	{
		float x = msg.ranges[i];
		float y = msg.ranges[j];
        if (x < eps || y < eps) return MAX_VALUE;
		float angle = (j - i) * msg.angle_increment;
        if (angle < 0) angle = -angle;
		return (x * x + y * y - 2 * x * y * quickCos(angle));
	}

public:
    int right[MAXN];
	int contours[MAXN]; 
    int cont_idx[MAXN];
    int n_cont_idx, m_conts;

	/*
	 * setting the parameters
	 */

	contour_extractor()
	{
		base_dis = PI * 1.5 / 1080;
	};


	void set_params(int _maxSkipPoints, double _adjacentAcceptDistance,
			double _maxJoinDistance, double _startContourMaxDistance,
			int _minPointsPerContour, double _maxDistanceRatio,
			double _alwaysAcceptDistance)
	{

		maxSkipPoints = _maxSkipPoints;

		adjacentAcceptDistance = _adjacentAcceptDistance;
		sqAdjacentAcceptDistance = sqr(adjacentAcceptDistance);

		maxJoinDistance = _maxJoinDistance;
		sqMaxJoinDistance = sqr(maxJoinDistance);

		startContourMaxDistance = _startContourMaxDistance;
		sqStartContourMaxDistance = sqr(startContourMaxDistance);

		minPointsPerContour = _minPointsPerContour;
		
		maxDistanceRatio = _maxDistanceRatio;
		sqMaxDistanceRatio = sqr(maxDistanceRatio);

		alwaysAcceptDistance = _alwaysAcceptDistance;
		sqAlwaysAcceptDistance = sqr(alwaysAcceptDistance);

        base_dis = PI * 1.5 / 1080;
        sqBaseDis = sqr(base_dis);
	}

	/*
	 * compute the contours
	 */
	void cmpt_contours(sensor_msgs::LaserScan _msg)
	{
        memset(cont_idx, 0, sizeof(cont_idx));
        memset(contours, 0, sizeof(contours));
        n_cont_idx = 0;
        m_conts = 0;

		gettimeofday(&tv, 0);
		double time_begin = tv.tv_sec * 1000000 + tv.tv_usec;

        msg = _msg;
		int number = msg.ranges.size(); //the laser beams in scan

		for (int i = 0; i < MAXN; i++)
            right[i] = -1;

		/* 1. get the right (possible closest point) array.*/
		int i, j;
		double tmp, now, lastSqDistance = sqStartContourMaxDistance, range;

		for (i = 0; i < number; i++)
        // get right closest (in a contour)
		{
            now = max(sqAlwaysAcceptDistance, lastSqDistance * sqMaxDistanceRatio);
        	for (j = i + 1; j <= i + maxSkipPoints && j < number; j++)
        		{
                	tmp = sqrDistance(i, j);
                	if (tmp < sqAdjacentAcceptDistance)
                	{
                    	right[i] = j;
                        now = tmp;
                    	break;
                	}
                    range = min(msg.ranges[i], msg.ranges[j]);
                	if (tmp < now)
                        if (tmp < sqMaxJoinDistance * sqr(range) * sqBaseDis * sqr(j - i))
                	    {
                    	   right[i] = j;
                    	   now = tmp;
                	    }
            	}
            if (right[i] < 0)
                lastSqDistance = sqStartContourMaxDistance;
            else
                lastSqDistance = now;
        }
        int c_tmp[MAXN], m;
        i = 0;
    	while (i < number)
    	{
        	m = 0;
        	memset(c_tmp, 0, sizeof(c_tmp));
        	c_tmp[m++] = i;
        	while (right[i] > 0)
            {
            	c_tmp[m++] = right[i];
                i = right[i];
            }
        	while (i < number && right[i] < 0)
                i++;
        	if (m < minPointsPerContour + 2 * CONT_TRIM) 
        		continue;

            cont_idx[n_cont_idx++] = m_conts;
        	for (j = CONT_TRIM; j < m - CONT_TRIM; j++)
            	contours[m_conts++] = c_tmp[j];
    	}
        cont_idx[n_cont_idx] = m_conts;

        if (DEBUG_OUTPUT)
        {

            gettimeofday(&tv, 0);
            double time_end = tv.tv_sec * 1000000 + tv.tv_usec;

            printf("contour_count:%3d | time_delay:%10f\n", n_cont_idx,
                    time_end - time_begin);
        }
	}

	/*
	 * get the msg in contour
	 */
	sensor_msgs::LaserScan get_contour_msg()
	{
		int number = msg.ranges.size(); //the laser beams in scan
		int flag[number];
		memset(flag, 0, sizeof(int) * number);

		for (int i = 0; i < m_conts; i++)
            flag[contours[i]] = 1;

		for (int i = 0; i < number; i++)
		{
			if (flag[i] == 0)
			{
				msg.ranges[i] = 0;
			}
		}

		return msg;
	}

    sensor_msgs::LaserScan get_anti_contour_msg()
    {
        int number = msg.ranges.size(); //the laser beams in scan
        int flag[number];
        memset(flag, 0, sizeof(int) * number);

        for (int i = 0; i < m_conts; i++)
            flag[contours[i]] = 1;

        for (int i = 0; i < number; i++)
        {
            if (flag[i] == 1)
            {
                msg.ranges[i] = 0;
            }
        }

        return msg;
    }

};

#endif
