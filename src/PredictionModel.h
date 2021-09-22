#ifndef _PREDICTION_MODEL_H
#define _PREDICTION_MODEL_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <cassert>

#include "utility.h"
#include "params.h"

// #include "DetectedVehicle.h"

class PredictionModel
{
public:
    void Import(const std::vector<std::vector<double> >& sensor_fusion);
    std::string map_file_ = "../data/highway_map.csv";

public:
  PredictionModel(std::vector<std::vector<double> > const &sensor_fusion, EgoVehicle const &car, int horizon);
  virtual ~PredictionModel();

  std::map<int, std::vector<Coord> > get_predictions() const { return m_mapPredictionsPath; };
  double GetSafetyDistance() const { return m_dSafetyDistance; };
  double get_paranoid_safety_distance() const { return m_dParanoidSafetyDistance; };
  double GetLaneSpeed(int lane) const;
  double GetLaneFreeSpace(int lane) const;


private:
  void SetSafetyDistances(std::vector<std::vector<double>> const &sensor_fusion, EgoVehicle const &car);
  void SetLaneInfo(std::vector<std::vector<double>> const &sensor_fusion, EgoVehicle const &car);
  std::vector<int> FindClosestVehicles(std::vector<std::vector<double>> const &sensor_fusion, EgoVehicle const &car);
  double GetSafetyDistance(double vel_back, double vel_front, double time_latency);

  // TODO use vector init depending on PARAM_NB_LANES
  std::vector<int> m_vFrontCarIndex= {-1, -1, -1};  // idx of closest object per lane
  std::vector<int> m_vBackCarIndex = {-1, -1, -1};   // idx of closest object per lane

  // TODO use FOV instead of INF
  std::vector<double> m_vFrontMinDist = {INF, INF, INF};  // dist min per lane
  std::vector<double> m_vBackMinDist = {INF, INF, INF};   // dist min per lane

  std::vector<double> m_vFrontVehicleVelocity = {PARAM_MAX_SPEED, PARAM_MAX_SPEED, PARAM_MAX_SPEED};
  std::vector<double> m_vFrontVehicleSafetyDistance = {PARAM_SD_LC, PARAM_SD_LC, PARAM_SD_LC};

  std::vector<double> m_vBackVehicleVelocity = {PARAM_MAX_SPEED, PARAM_MAX_SPEED, PARAM_MAX_SPEED};
  std::vector<double> m_vBackVehicleSafetyDistance = {PARAM_SD_LC, PARAM_SD_LC, PARAM_SD_LC};

  // map of at most 6 predicitons of "n_horizon" (x,y) coordinates
  std::map< int, std::vector<Coord> > m_mapPredictionsPath;
  double m_vLaneSpeed[PARAM_NB_LANES];
  double m_vLaneFreeSpace[PARAM_NB_LANES];

  // safety distance computation related
  double m_dEgoVel;
  double m_dDecel;

  double m_dFrontVehicleDistance;
  double m_dFrontVehicleVel;
  double m_tmToCollision;  // vs front vehicle
  double m_tmToStop;  // time from vel_ego_ to 0
  double m_tmToDecelerate;  // time from vel_ego_ to vel_front_

  // will be used by behavior planner
  double m_dSafetyDistance = PARAM_SD;
  double m_dParanoidSafetyDistance = PARAM_SD;

private:
#if 0
  std::map<VID, DetectedVehicle*> m_mapDetectedVehicles;
#endif

};

#endif // _PREDICTION_MODEL_H
