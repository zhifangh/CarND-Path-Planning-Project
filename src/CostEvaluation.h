#ifndef _COST_EVALUATION_
#define _COST_EVALUATION_

#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <cassert>
#include <cmath>

#include "utility.h"
#include "params.h"
#include "PredictionModel.h"
#include "Eigen-3.3/Eigen/Dense"

#include "BehaviorPlanner.h"
#include "TrajectoryGenerator.h"

class CostEvaluation
{
public:
    CostEvaluation(struct TrajectoryXY const &trajectory, TargetConfig target, PredictionModel &predictions, int car_lane);
    virtual ~CostEvaluation();

    double GetCost();

    double GetCost() const;
    void SetCost(double GetCost);

private:
    bool CheckCollision(double x0, double y0, double theta0, double x1, double y1, double theta1);
    int  CheckCollisionOnTrajectory(struct TrajectoryXY const &trajectory, std::map<int, std::vector<Coord> > &predictions);

#if 0
    bool CheckMaxCapabilities(std::vector<std::vector<double>> &traj);
    double GetPredictedMinDistance(struct TrajectoryXY const &trajectory, std::map<int, std::vector<Coord> > &predictions);
#endif

    double m_dCost;
};

#endif // _COST_EVALUATION_
