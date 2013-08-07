#include "get_contour2.h"

inline double ContourExtractor::sqrDistance(int i, int j)
{
    float x = msg.ranges[i];
    float y = msg.ranges[j];
    if (x < eps || y < eps) 
        return MAX_VALUE;
    float angle = (j - i) * msg.angle_increment;
    if (angle < 0) angle = -angle;
    return (x * x + y * y - 2 * x * y * quickCos(angle));
}

ContourExtractor::ContourExtractor()
{
    base_dis = PI * 1.5 / 1080;
};

/* setting the parameters */

void ContourExtractor::set_params(int _maxSkipPoints, int _endTrimPoints, double _adjacentAcceptDistance, double _maxJoinDistance, double _startContourMaxDistance, int _minPointsPerContour, double _maxDistanceRatio, double _alwaysAcceptDistance)
{

    maxSkipPoints = _maxSkipPoints;
    endTrimPoints = _endTrimPoints;

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

/* compute the contours */

void ContourExtractor::compute_contours(sensor_msgs::LaserScan _msg)
{
    memset(cont_idx, 0, sizeof(cont_idx));
    memset(contours, 0, sizeof(contours));
    n_cont_idx = 0;
    m_conts = 0;

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
        now = std::max(sqAlwaysAcceptDistance, lastSqDistance * sqMaxDistanceRatio);
        for (j = i + 1; j <= i + maxSkipPoints && j < number; j++)
            {
                tmp = sqrDistance(i, j);
                if (tmp < sqAdjacentAcceptDistance)
                {
                    right[i] = j;
                    now = tmp;
                    break;
                }
                range = std::min(msg.ranges[i], msg.ranges[j]);
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
        if (m < minPointsPerContour + 2 * endTrimPoints) 
            continue;

        cont_idx[n_cont_idx++] = m_conts;
        for (j = endTrimPoints; j < m - endTrimPoints; j++)
            contours[m_conts++] = c_tmp[j];
    }
    cont_idx[n_cont_idx] = m_conts;
}

/* get the msg in contour. It will ruin your msg, use it only once after computing. */

sensor_msgs::LaserScan ContourExtractor::get_contour_msg(int reverse)
{
    int number = msg.ranges.size(); //the laser beams in scan
    int flag[number];
    memset(flag, 0, sizeof(int) * number);
    if (reverse) reverse = 1;

    for (int i = 0; i < m_conts; i++)
        flag[contours[i]] = 1;

    for (int i = 0; i < number; i++)
    {
        if (flag[i] == reverse)
        {
            msg.ranges[i] = 0;
        }
    }

    return msg;
}
