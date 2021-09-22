#include "BehaviorPlanner.h"

using namespace std;

/*
 * behavior planner will provide a list of candidate targets rather than a single suggested maneuver.
*/
BehaviorPlanner::BehaviorPlanner(vector<vector<double>> const &sensor_fusion, EgoVehicle car, PredictionModel const &predictionModel)
{
    TargetConfig target;
    target.time = 2.0;
    double car_speed_target = car.speed_target;

    double safety_distance = predictionModel.GetSafetyDistance();

    if (car.emergency)
        car_speed_target = car.speed;

    bool too_close = false;
    int ref_vel_inc = 0; // -1 for max deceleration, 0 for constant speed, +1 for max acceleration

    double ref_vel_ms = mph_to_ms(car_speed_target);
    double closest_speed_ms = PARAM_MAX_SPEED;
    double closest_dist = INF;

    // find ref_v to use based on car in front of us
    for (size_t i = 0; i < sensor_fusion.size(); i++)
    {
        // car is in my lane
        float d = sensor_fusion[i][6];
        if (d > GetLeftBoundary(car.lane) && d < GetRightBoundary(car.lane))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];

            if ((check_car_s > car.s) && ((check_car_s - car.s) < safety_distance))
            {
                too_close = true;
                double dist_to_check_car_s = check_car_s - car.s;
                if (dist_to_check_car_s < closest_dist)
                {
                    closest_dist = dist_to_check_car_s;
                    closest_speed_ms = check_speed;
                }
            }
        }
    }

    if (too_close)
    {
        // slow down
        if (ref_vel_ms > closest_speed_ms)
        {
            car_speed_target -= PARAM_MAX_SPEED_INC_MPH;
            if (closest_dist <= 10 && car_speed_target > closest_speed_ms)
            {
                car_speed_target -= 5 * PARAM_MAX_SPEED_INC_MPH;
            }
        }

        car_speed_target = max(car_speed_target, 0.0);

        ref_vel_inc = -1;
    }
    else if (car_speed_target < PARAM_MAX_SPEED_MPH)
    {
        // speed up
        car_speed_target += PARAM_MAX_SPEED_INC_MPH;
        car_speed_target = min(car_speed_target, PARAM_MAX_SPEED_MPH);

        ref_vel_inc = +1;
    }

    // our nominal target .. same lane
    target.lane = car.lane;
    target.velocity = car_speed_target;

    /*
     * The first possible target will relate to adjusting the speed in our lane and keeping basically a rather big 30 meters distance with the vehicule in front of us;
     * we end up driving at the same speed, 30 meters behind the vehicule in front of us.
    */
    m_vTargetConfig.push_back(target);

    /*
     * Then other targets are proposed: for every other adjacent lane.
     * Then for every lane target, the behavior planner proposes a speed target corresponding to either our current lane speed target or a slower speed.
    */
    // Backup targets (lane and speed)
    vector<int> backup_lanes;
    switch (car.lane)
    {
    case 2:
        backup_lanes.push_back(1); // Lane 2 -> Lane 1
        break;
    case 1:
        backup_lanes.push_back(2); // Lane 1 -> Lane 2
        backup_lanes.push_back(0); // Lane 1 -> Lane 0
        break;
    case 0:
        backup_lanes.push_back(1); // Lane 0 -> Lane 1
        break;
    default:
        assert(1 == 0); // something went wrong
        break;
    }

    vector<double> backup_vel; // only lower speeds so far ...
    switch (ref_vel_inc)
    {
    case 1:
        backup_vel.push_back(car_speed_target - PARAM_MAX_SPEED_INC_MPH);
        backup_vel.push_back(car_speed_target - 2 * PARAM_MAX_SPEED_INC_MPH);
        break;
    case 0: // already max speed
        backup_vel.push_back(car_speed_target - PARAM_MAX_SPEED_INC_MPH);
        break;
    case -1:
        // emergency breaking
        backup_vel.push_back(car_speed_target - PARAM_MAX_SPEED_INC_MPH);
        break;
    default:
        assert(1 == 0); // something went wrong
        break;
    }

    // 1) backup velocities on target lane
    target.lane = car.lane;
    for (size_t i = 0; i < backup_vel.size(); i++)
    {
        target.velocity = backup_vel[i];
        m_vTargetConfig.push_back(target);
    }

    // 2) target velocity on backup lanes
    target.velocity = car_speed_target;
    for (size_t i = 0; i < backup_lanes.size(); i++)
    {
        target.lane = backup_lanes[i];
        m_vTargetConfig.push_back(target);
    }

    // 2) backup velocities on backup lanes
    for (size_t i = 0; i < backup_vel.size(); i++)
    {
        target.velocity = backup_vel[i];
        for (size_t j = 0; j < backup_lanes.size(); j++) {
            target.lane = backup_lanes[j];
            m_vTargetConfig.push_back(target);
        }
    }

    // Last target/candidate: emergency trajectory (just in case we have no better choice)
    target.lane = car.lane;
    target.velocity = predictionModel.GetLaneSpeed(car.lane);
    target.time = 0.0; // ASAP ... (identified as emergency target)
    target.accel = -0.85 * PARAM_MAX_ACCEL;
    m_vTargetConfig.push_back(target);
}

BehaviorPlanner::~BehaviorPlanner()
{

}

vector<TargetConfig> BehaviorPlanner::GetTargetConfig()
{
    return m_vTargetConfig;
}
