#include "PredictionModel.h"

using namespace std;

/*
 * First we look for the closest objects: the car just in front of the ego vehicle and the car just behind the ego vehicle for every lane;
 * assuming a Field Of View of 70 meters, front and back (which is a configurable parameter).
 *
 * These predictions will be used later on to check for potential collisions and safety distances and evaluate whether a lane is a good target or not e.g.
 * depending on the presence and/or speed of cars in these lanes.
*/
PredictionModel::PredictionModel(vector<vector<double>> const &sensor_fusion, EgoVehicle const &car, int horizon)
{
    vector<int> vClosestVehicles = FindClosestVehicles(sensor_fusion, car);

    for (int i = 0; i < vClosestVehicles.size(); i++)
    {
        int fusion_index = vClosestVehicles[i];
        if (fusion_index >= 0) // exclude -1
        {
            double x = sensor_fusion[fusion_index][1];
            double y = sensor_fusion[fusion_index][2];
            double vx = sensor_fusion[fusion_index][3];
            double vy = sensor_fusion[fusion_index][4];
            vector<Coord> prediction;

            // We generate the trajectory over a 1 second horizon to check for potential future collisions.
            for (int j = 0; j < horizon; j++)
            {
                Coord coord;
                coord.x = x + vx * j * PARAM_DT;
                coord.y = y + vy * j * PARAM_DT;
                prediction.push_back(coord);
            }
            m_mapPredictionsPath[fusion_index] = prediction;
        }
    }

    SetSafetyDistances(sensor_fusion, car);
    SetLaneInfo(sensor_fusion, car);
}

PredictionModel::~PredictionModel()
{
#if 0

    for (auto it = m_mapDetectedVehicles.begin(); it != m_mapDetectedVehicles.end(); ++it)
    {
        delete it->second;
    }

    m_mapDetectedVehicles.clear();

#endif

}

// vx,vy => v
double GetVehicleVel(vector<vector<double>> const &sensor_fusion, int idx, double default_vel)
{
    double vx, vy, vel;
    if (idx >= 0 && idx < sensor_fusion.size())
    {
        vx = sensor_fusion[idx][3];
        vy = sensor_fusion[idx][4];
        vel = sqrt(vx * vx + vy * vy);
    }
    else
    {
        vel = default_vel;
    }

    return vel;
}

// two vehicle safe distance
double PredictionModel::GetSafetyDistance(double vel_back, double vel_front, double time_latency)
{
    //cout << "get_safety_distance: " << vel_back << ", " << vel_front << ", " << time_latency << '\n';

    double safety_distance = PARAM_SD_LC;
    if (vel_back > vel_front)
    {
        double time_to_decelerate = (vel_back - vel_front) / m_dDecel + time_latency;
        safety_distance = vel_back * time_to_decelerate + 1.5 * PARAM_CAR_SAFETY_L;
    }
    safety_distance = max(safety_distance, PARAM_SD_LC);  // conservative

    return safety_distance;
}

// calculate ego and it's surrouding vehicle's safe distance
void PredictionModel::SetSafetyDistances(vector<vector<double>> const &sensor_fusion, EgoVehicle const &car)
{
    m_dEgoVel = mph_to_ms(car.speed);  // velocity of ego vehicle
    // slightly conservative as it will relate to safety distance
    m_dDecel = 0.8 * PARAM_MAX_ACCEL;
    m_tmToStop = m_dEgoVel / m_dDecel;

    m_dFrontVehicleVel = GetVehicleVel(sensor_fusion, m_vFrontCarIndex[car.lane], PARAM_MAX_SPEED);
    m_dFrontVehicleDistance = m_vFrontMinDist[car.lane];

    // ego safe distance
    if (m_dEgoVel > m_dFrontVehicleVel)
    {
        m_tmToCollision = m_dFrontVehicleDistance / (m_dEgoVel - m_dFrontVehicleVel);
        m_tmToDecelerate = (m_dEgoVel - m_dFrontVehicleVel) / m_dDecel;
        m_dSafetyDistance = m_dEgoVel * m_tmToDecelerate + 1.75 * PARAM_CAR_SAFETY_L;
    }
    else
    {
        m_tmToCollision = INF;
        m_tmToDecelerate = 0;
        m_dSafetyDistance = 1.75 * PARAM_CAR_SAFETY_L;
    }

    m_dParanoidSafetyDistance = m_dEgoVel * m_tmToStop + 2 * PARAM_CAR_SAFETY_L;

    cout << "SAFETY: D=" << m_dFrontVehicleDistance << " dV=" << m_dEgoVel - m_dFrontVehicleVel << " TTC=" << m_tmToCollision
       << " TTD=" << m_tmToDecelerate << " SD=" << m_dSafetyDistance << " PSD=" << m_dParanoidSafetyDistance << '\n';

    // surrouding vehices's safe distance
    for (int i = 0; i < PARAM_NB_LANES; i++)
    {
        m_vFrontVehicleVelocity[i] = GetVehicleVel(sensor_fusion, m_vFrontCarIndex[i], PARAM_MAX_SPEED);
        m_vFrontVehicleSafetyDistance[i] = GetSafetyDistance(m_dEgoVel, m_vFrontVehicleVelocity[i], 0.0);

        m_vBackVehicleVelocity[i] = GetVehicleVel(sensor_fusion, m_vBackCarIndex[i], 0);
        m_vBackVehicleSafetyDistance[i] = GetSafetyDistance(m_vBackVehicleVelocity[i], m_dEgoVel, 2.0);

        cout << "SAFETY_DISTANCE for LC[" << i << "]: front_sd=" << m_vFrontVehicleSafetyDistance[i] << " back_sd=" << m_vBackVehicleSafetyDistance[i] << '\n';
    }
}

// calculate ego's possible speed and free space for each lane
void PredictionModel::SetLaneInfo(vector<vector<double>> const &sensor_fusion, EgoVehicle const &car)
{
    int car_lane = GetLaneIndex(car.d);
    for (size_t i = 0; i < m_vFrontCarIndex.size(); i++)
    {
        cout << "lane " << i << ": ";
        cout << "front " << m_vFrontCarIndex[i] << " at " << m_vFrontMinDist[i] << " s_meters ; ";
        cout << "back " << m_vBackCarIndex[i] << " at " << m_vBackMinDist[i] << " s_meters" << endl;

        int lane = i;
        if (m_vFrontCarIndex[i] >= 0)
        {
            // a car in front of us
            if (lane != car_lane && (m_vBackMinDist[i] <= m_vBackVehicleSafetyDistance[i] || m_vFrontMinDist[i] <= m_vFrontVehicleSafetyDistance[i]))
            {
                // distance is less than saft distance
                m_vLaneSpeed[i] = 0;
                m_vLaneFreeSpace[i] = 0; // too dangerous
            }
            else
            {
                // distance is more than safe distance
                double vx = sensor_fusion[m_vFrontCarIndex[i]][3];
                double vy = sensor_fusion[m_vFrontCarIndex[i]][4];
                m_vLaneSpeed[i] = sqrt(vx * vx + vy * vy);
                m_vLaneFreeSpace[i] = m_vFrontMinDist[i];
            }
        }
        else
        {
            // if nobody in front of us
            if (lane != car_lane && m_vBackMinDist[i] <= m_vBackVehicleSafetyDistance[i])
            {
                m_vLaneSpeed[i] = 0;
                m_vLaneFreeSpace[i] = 0; // too dangerous
            }
            else
            {
                m_vLaneSpeed[i] = PARAM_MAX_SPEED_MPH;
                m_vLaneFreeSpace[i] = PARAM_FOV;
            }
        }

        cout << "Predictions::lane_speed_[" << i << "]=" << m_vLaneSpeed[i] << endl;
    }
}

// we generate predictions for closet car per lane in front of us
// we generate predictions for closet car per lane behind us
// => at most 6 predictions (for now on) as we have 3 lanes

// Find closest car in both direction: forward and backward for each lane
vector<int> PredictionModel::FindClosestVehicles(vector<vector<double>> const &sensor_fusion, EgoVehicle const &car)
{
    // Handle FOV and s wraparound
    double sfov_min = car.s - PARAM_FOV;
    double sfov_max = car.s + PARAM_FOV;
    double sfov_shit = 0;

    if (sfov_min < 0)
    {
        // Handle s wrapping
        sfov_shit = -sfov_min;
    }
    else if (sfov_max > MAX_S)
    {
        sfov_shit = MAX_S - sfov_max;
    }

    sfov_min += sfov_shit;
    sfov_max += sfov_shit;

    assert(sfov_min >= 0 && sfov_min <= MAX_S);
    assert(sfov_max >= 0 && sfov_max <= MAX_S);

    double car_s = car.s;

    car_s += sfov_shit;

    for (size_t i = 0; i < sensor_fusion.size(); i++)
    {
        double s = sensor_fusion[i][5] + sfov_shit;
        if (s >= sfov_min && s <= sfov_max)
        {
            // object in FOV
            double d = sensor_fusion[i][6];
            int lane = GetLaneIndex(d);
            if (lane < 0 || lane > 2)
              continue; // some garbage values in sensor_fusion from time to time

            // s wraparound already handled via FOV shift
            //double dist = get_sdistance(s, car_s);
            double dist = fabs(s - car_s);

            if (s >= car_s)
            {
                // front
                if (dist < m_vFrontMinDist[lane])
                {
                    m_vFrontCarIndex[lane] = i;
                    m_vFrontMinDist[lane] = dist;
                }
            }
            else
            {
                // back
                if (dist < m_vBackMinDist[lane])
                {
                    m_vBackCarIndex[lane] = i;
                    m_vBackMinDist[lane] = dist;
                }
            }
        }
    }

    return { m_vFrontCarIndex[0], m_vBackCarIndex[0], m_vFrontCarIndex[1], m_vBackCarIndex[1], m_vFrontCarIndex[2], m_vBackCarIndex[2] };
}

double PredictionModel::GetLaneSpeed(int lane) const
{
    if (lane >= 0 && lane <= 3)
    {
        return m_vLaneSpeed[lane];
    }
    else
    {
        return 0;
    }
}

double PredictionModel::GetLaneFreeSpace(int lane) const
{
    if (lane >= 0 && lane <= 3)
    {
        return m_vLaneFreeSpace[lane];
    }
    else
    {
        return 0;
    }
}

#if 0
void PredictionModel::Import(const std::vector<std::vector<double> >& sensor_fusion)
{
    for (size_t i = 0; i < sensor_fusion.size(); i++)
    {
        auto pDetectedVehicle = new DetectedVehicle();
        pDetectedVehicle->id    = i;

        pDetectedVehicle->x     = sensor_fusion[i][1];
        pDetectedVehicle->y     = sensor_fusion[i][2];
        pDetectedVehicle->v_x   = sensor_fusion[i][3];
        pDetectedVehicle->v_y   = sensor_fusion[i][4];
        pDetectedVehicle->s     = sensor_fusion[i][5];

        m_mapDetectedVehicles.insert(make_pair(pDetectedVehicle->id, pDetectedVehicle));
    }
}
#endif
