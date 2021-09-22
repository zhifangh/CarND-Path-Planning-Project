#include "utility.h"
#include "params.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <math.h>

#include "map.h"

#include <time.h>

using namespace std;

double MAX_S;

/**
 * Initializes Vehicle
 */
Map::Map(std::string map_file)
{
    Import(map_file);
};

Map::~Map()
{

}

void Map::Import(string map_file)
{
    ifstream isMap(map_file.c_str(), ifstream::in);
    string line;

    double last_s = 0;

    while (getline(isMap, line))
    {
        istringstream iss(line);

        double x, y;
        float s, d_x, d_y;

        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;

        m_waypointsX.push_back(x);
        m_waypointsY.push_back(y);
        m_waypointsS.push_back(s);
        last_s = s;
        m_waypointsDx.push_back(d_x);
        m_waypointsDy.push_back(d_y);
    }
    assert(m_waypointsX.size() && "map not loaded, probably path include is missing");

    MAX_S = last_s;

    m_splineX.set_points(m_waypointsS, m_waypointsX);
    m_splineY.set_points(m_waypointsS, m_waypointsY);
    m_splineDx.set_points(m_waypointsS, m_waypointsDx);
    m_splineDy.set_points(m_waypointsS, m_waypointsDy);
}

int Map::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
    double dClosestLength = std::numeric_limits<double>::max ();
    int iClosestWaypointIndex = 0;

    int size = maps_x.size();
    for (int i = 0; i < size; i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if(dist < dClosestLength)
        {
            dClosestLength = dist;
            iClosestWaypointIndex = i;
        }
    }

    return iClosestWaypointIndex;
}

int Map::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int iClosestWaypointIndex = ClosestWaypoint(x,y,maps_x,maps_y);

    double dClosestWaypointX = maps_x[iClosestWaypointIndex];
    double dClosestWaypointY = maps_y[iClosestWaypointIndex];

    double heading = atan2((dClosestWaypointY-y), (dClosestWaypointX-x));

	double angle = fabs(theta-heading);
    angle = min(2 * M_PI - angle, angle);

	if(angle > M_PI/4)
	{
        // (x, y) is before (dClosestWaypointX, dClosestWaypointY)
        iClosestWaypointIndex++;
        if (iClosestWaypointIndex == maps_x.size())
        {
          iClosestWaypointIndex = 0; // loop
        }
	}

    return iClosestWaypointIndex;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta)
{
    vector<double> &maps_s = this->m_waypointsS;
    vector<double> &maps_x = this->m_waypointsX;
    vector<double> &maps_y = this->m_waypointsY;

    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size() - 1; // loop
    }

    double total_x = maps_x[next_wp] - maps_x[prev_wp];
    double total_y = maps_y[next_wp] - maps_y[prev_wp];
    double prev_x = x - maps_x[prev_wp];
    double prev_y = y - maps_y[prev_wp];

    // find the projection
    double proj_norm = (prev_x * total_x + prev_y * total_y) / (total_x * total_x + total_y * total_y);
    double proj_x = proj_norm * total_x;
    double proj_y = proj_norm * total_y;

    double frenet_d = distance(prev_x, prev_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point
    double frenet_s = maps_s[prev_wp];
    frenet_s += distance(0, 0, proj_x, proj_y);

    assert(frenet_d >= 0);

    return {frenet_s, frenet_d};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d)
{
    vector<double> &maps_s = m_waypointsS;
    vector<double> &maps_x = m_waypointsX;
    vector<double> &maps_y = m_waypointsY;

    int prev_wp = -1;

    while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - M_PI/2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

vector<double> Map::getXYspline(double s, double d)
{
    s = fmod(s, MAX_S);
    double x = m_splineX(s) + d * m_splineDx(s);
    double y = m_splineY(s) + d * m_splineDy(s);

    return {x,y};
}

double Map::getSpeedToFrenet(double Vxy, double s)
{
    s = fmod(s, MAX_S);
    double dx_over_ds = m_splineX.deriv(1, s);
    double dy_over_ds = m_splineY.deriv(1, s);
    double Vs = (Vxy / sqrt(dx_over_ds * dx_over_ds + dy_over_ds * dy_over_ds));
    return Vs;
}
