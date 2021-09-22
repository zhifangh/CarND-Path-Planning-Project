#ifndef _BEHAVIOR_PLANNER_H
#define _BEHAVIOR_PLANNER_H

#include <math.h>
#include <iostream>
#include <vector>
#include <cassert>

#include "params.h"
#include "utility.h"
#include "PredictionModel.h"

struct TargetConfig
{
    double lane;
    double velocity;    // for JMT trajectories
    double time;        // for manoeuver
    double accel;       // for emergency trajectories

public:
    TargetConfig(double l=0, double v=0, double t=0, double a=0) : lane(l), velocity(v), time(t), accel(a) {}
};


class BehaviorPlanner
{
public:
    BehaviorPlanner(std::vector<std::vector<double>> const &sensor_fusion, EgoVehicle car, PredictionModel const &predictionModel);
    virtual ~BehaviorPlanner();
    std::vector<TargetConfig> GetTargetConfig();

private:
    std::vector<TargetConfig> m_vTargetConfig;
};


#endif // _BEHAVIOR_PLANNER_H
