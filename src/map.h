#ifndef MAP_H
#define MAP_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "spline.h"

class Map {

public:
  /**
  * Constructor
  */
  Map(std::string map_file);
  
  /**
  * Destructor
  */
  virtual ~Map();

public:
  std::vector<double> getFrenet(double x, double y, double theta);
  std::vector<double> getXY(double s, double d);
  std::vector<double> getXYspline(double s, double d); // with splines
  double getSpeedToFrenet(double Vxy, double s);

private:
  void Import(std::string map_file);

  int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
  int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

private:
  tk::spline m_splineX;
  tk::spline m_splineY;
  tk::spline m_splineDx;
  tk::spline m_splineDy;

  std::vector<double> m_waypointsX;
  std::vector<double> m_waypointsY;
  std::vector<double> m_waypointsS;
  std::vector<double> m_waypointsDx;
  std::vector<double> m_waypointsDy;
};

#endif // MAP_H
