# CarND-Path-Planning-Project

## **Goals**

The goals of this project are the following:

* Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit..

* Passing slower traffic when possible

* The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.

* The car should be able to make one complete loop around the 6946m highway. 

* The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

  

### Pipeline 

### Input

#### 1. The map of the highway is in data/highway_map.csv

Each waypoint in the list contains  [x,y,s,dx,dy] values. 

-  x and y are the waypoint's map coordinate position, 
- s value is the distance along the road to get to that waypoint in meters, 
- dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.



#### 2. Main car's localization Data (No Noise)

- ["x"] The car's x position in map coordinates
- ["y"] The car's y position in map coordinates

- ["s"] The car's s position in frenet coordinates

- ["d"] The car's d position in frenet coordinates
- ["yaw"] The car's yaw angle in the map
- ["speed"] The car's speed in MPH



#### 3. Previous path data given to the Planner

- ["previous_path_x"] The previous list of x points previously given to the simulator
- ["previous_path_y"] The previous list of y points previously given to the simulator



#### 4. Previous path's end s and d values 

- ["end_path_s"] The previous list's last point's frenet s value
- ["end_path_d"] The previous list's last point's frenet d value



#### 5. Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's attributes:

- car's unique ID,
- car's x position in map coordinates,
- car's y position in map coordinates,
- car's x velocity in m/s,
- car's y velocity in m/s, 
- car's s position in frenet coordinates,
- car's d position in frenet coordinates.



### Process

#### 1. Prediction Model

Based on fusion data, looking for the closest vehicles with the ego car.  These vehicles are in the font of or behind of the ego car for each lane, so at most six vehicles are traced.

For each traced vehilce, record the distance with ego car, generate a trajectory over a 1 second horizon (50 points, 1 point every 0.02s) to check for potential future collisions based on the vehilce's velocity.

```c++
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
```
Calculate safe distance between ego car and it's surrouding vehicles.

```c++
// surrouding vehices's safe distance
for (int i = 0; i < PARAM_NB_LANES; i++)
{
    m_vFrontVehicleVelocity[i] = GetVehicleVel(sensor_fusion, m_vFrontCarIndex[i], PARAM_MAX_SPEED);
    m_vFrontVehicleSafetyDistance[i] = GetSafetyDistance(m_dEgoVel, m_vFrontVehicleVelocity[i], 0.0);

    m_vBackVehicleVelocity[i] = GetVehicleVel(sensor_fusion, m_vBackCarIndex[i], 0);
    m_vBackVehicleSafetyDistance[i] = GetSafetyDistance(m_vBackVehicleVelocity[i], m_dEgoVel, 2.0);

    cout << "SAFETY_DISTANCE for LC[" << i << "]: front_sd=" << m_vFrontVehicleSafetyDistance[i] << " back_sd=" << m_vBackVehicleSafetyDistance[i] << '\n';
}
```
Calculate ego car's possible speed and free space for each lane. If the disance between the traced vehicle and the ego car is less than safe distance, mark it as dangerous. Otherwise, if there is a vehicle in front of  ego car, ego car can drive a safe distance at the speed of the front vehicle, and if there is no vehicle in front of ego car, ego car can drive a safe distance at the maximum speed.  

```c++
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
```



#### 2. Behavior Planner

Check the vehicle in the same lane and in front of the ego car,  judge whether it is too close to the ego car. If it is too close, ego car will slow down; otherwise, ego car will accelerate and the speed is not over the maximum speed.  

```c++
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
```
Consider various possible vehicle operating conditions:  

1. Keep in the current lane at a reasonable speed  

2. Lane change: Lane change to the left or right  

3. Change speed: Driving at a possible speed lower than the target speed  

4. Emergency treatment  

 In this way, a group of possible driving target states of the vehicle is formed for evaluation and selection.  



#### 3. Trajectory Generator

The driving trajectory is generated according to the possible driving target states of the ego car.  

In order to keep the continuity of the vehicle trajectory, a few unused points from the last vehicle trajectory will be retained and other points will be regenerated.  

JMT trajectory is generated during normal operation, and JMT optimization is not carried out during emergency treatment. 

##### Trajectory generation for emergency treatment:  

Calculate the following parameters:  

double s, 			  // Start longitudinal distance  

ouble  s_dot, 		// Star longitudinal velocity  

ouble  s_ddot;	   // Start longitudinal acceleration  

 

double d, 			// Start laterally distance  

ouble  d_dot, 	 // Start laterally velocity  

ouble d_ddot;	 // Start laterally accelerated  



According to the distance of the starting point (s), combined with the target velocity, target acceleration, calculate s, velocity, acceleration of each point. d keep the same value. 

```c++
double s, s_dot, s_ddot;
double d, d_dot, d_ddot;
if (prev_size > 0)
{
    // exist not used points.
    for (int i = 0; i < prev_size; i++)
    {
        // not use points in previous cycle
        new_path_s[i] = previous_path_s[PARAM_NB_POINTS - previous_path_x.size() + i];
        new_path_d[i] = previous_path_d[PARAM_NB_POINTS - previous_path_x.size() + i];

        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // initial conditions for new (s,d) trajectory
    s = new_path_s[prev_size - 1].f;
    s_dot = new_path_s[prev_size - 1].f_dot;
    d = new_path_d[prev_size - 1].f;
    d_dot = 0;
    d_ddot = 0;
}
else
{
    // all points re-generate.
    s = car.s;
    s_dot = car.speed;
    d = car.d;
    d_dot = 0;
    d_ddot = 0;
}

s_ddot = target.accel;

// continuity point reused
double t = PARAM_DT; // dt
double prev_s_dot = s_dot; // v
for (int i = prev_size; i < PARAM_NB_POINTS; i++)
{
    // add new points exclude retained in last cycle.
    // increase/decrease speed till target velocity is reached
    s_dot += s_ddot * PARAM_DT; // v += acc * dt
    if ((target.accel > 0 && prev_s_dot <= target_velocity_ms && s_dot > target_velocity_ms)
     || (target.accel < 0 && prev_s_dot >= target_velocity_ms && s_dot < target_velocity_ms))
    {
        s_dot = target_velocity_ms; // not over target vel.
    }
    s_dot = max(min(s_dot, 0.9 * PARAM_MAX_SPEED), 0.0); // [0, 0.9 * PARAM_MAX_SPEED] // v
    s += s_dot * PARAM_DT; // s += v * dt

    prev_s_dot = s_dot;

    // SD
    new_path_s[i] = PointC2(s, s_dot, s_ddot);
    new_path_d[i] = PointC2(d, d_dot, d_ddot);

    // XY
    vector<double> point_xy = map.getXYspline(s, d);

    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);

    t += PARAM_DT;
}
```
##### Trajectory generation for normal driving:  

Calculate the following parameters:  

double si;	  		// Start longitudinal distance  

double si_dot;  	// Start longitudinal speed  

double si_ddot;   // Start longitudinal acceleration  

double di;	  	  // Start laterally distance  

double di_dot;    // Start laterally velocity  

double di_ddot;  // Start laterally accelerated  

 

double sf;	  		// End longitudinal distance  

double sf_dot;  	// End longitudinal velocity  

double sf_ddot;   // End longitudinal acceleration  

double df;	  	  // End laterally  distance  

double df_dot;    // End laterally velocity  

double df_ddot;  // End laterally accelerated  



After the above parameters are calculated, JMT function is called to obtain the coefficient of the sampling line function. regenerate remaining part by sampling line function.  

```c++
vector<double> start_s = { si, si_dot, si_ddot};
vector<double> end_s = { sf, sf_dot, 0};

vector<double> start_d = { di, di_dot, di_ddot };
vector<double> end_d = { df, df_dot, df_ddot};

// do a matrix inversion to derive our 6 unknown coefficients by JMT polynomial solver.
vector<double> poly_s = JMT(start_s, end_s, T);
vector<double> poly_d = JMT(start_d, end_d, T);

vector<double> next_x_vals;
vector<double> next_y_vals;

for (int i = 0; i < prev_size; i++)
{
    // reuse points in previous cycle
    new_path_s[i] = previous_path_s[PARAM_NB_POINTS - previous_path_x.size() + i];
    new_path_d[i] = previous_path_d[PARAM_NB_POINTS - previous_path_x.size() + i];

    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
}

// continuity point reused
double t = PARAM_DT;
for (int i = prev_size; i < PARAM_NB_POINTS; i++)
{
    // add new points exclude retained in last cycle.
    double s = polyeval(poly_s, t);
    double s_dot = polyeval_dot(poly_s, t);
    double s_ddot = polyeval_ddot(poly_s, t);

    double d = polyeval(poly_d, t);
    double d_dot = polyeval_dot(poly_d, t);
    double d_ddot = polyeval_ddot(poly_d, t);

    new_path_s[i] = PointC2(s, s_dot, s_ddot);
    new_path_d[i] = PointC2(d, d_dot, d_ddot);
```


```c++
    // Once the s(t) and d(t) have been found for the trajectory,
    // we convert back to (x, y) coordinates using the accurate getXYspline function:
    // we want to check speeed and acceleration in cartesian coordinates.
    vector<double> point_xy = map.getXYspline(s, d);

    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);

    t += PARAM_DT;
}
```


#### 4. Cost Evaluation

For each possible driving  trajectory , cost evaluate is carried out based on feasibility, safety, legality, comfort, efficiency.  

Priority and weight order: feasibility > safety > legitimacy > comfort > efficiency  

The most important is collision detection, whether the trajectory of the ego car and the trajectory of  the surrounding traced vehicles will collide in a certain period of time. If there is conflict, the cost is high. 

```c++
CostEvaluation::CostEvaluation(TrajectoryXY const &trajectory, TargetConfig target, PredictionModel &predict, int car_lane)
{
    m_dCost = 0; // lower cost preferred

    double cost_feasibility = 0;    // vs collisions, vs vehicle capabilities
    double cost_safety      = 0;    // vs buffer distance, vs visibility
    double cost_legality    = 0;    // vs speed limits
    double cost_comfort     = 0;    // vs jerk
    double cost_efficiency  = 0;    // vs desired lane and time to goal

    std::map<int, vector<Coord> > predictions = predict.get_predictions();

    // 1) FEASIBILITY cost
    if (CheckCollisionOnTrajectory(trajectory, predictions))
    {
        cost_feasibility += 10;
    }

    vector<vector<double>> traj;
    traj.push_back(trajectory.x_vals);
    traj.push_back(trajectory.y_vals);
    if (CheckMaxCapabilities(traj))
    {
        cost_feasibility += 1;
    }

    m_dCost = m_dCost + PARAM_COST_FEASIBILITY * cost_feasibility;

    // 2) SAFETY cost
    double dmin = GetPredictedMinDistance(trajectory, predictions);
    if (dmin < predict.GetSafetyDistance())
    {
        cost_safety = predict.GetSafetyDistance() - dmin;
    }

    m_dCost = m_dCost + PARAM_COST_SAFETY * cost_safety;

    // 3) LEGALITY cost
    m_dCost = m_dCost + PARAM_COST_LEGALITY * cost_legality;

    // 4) COMFORT cost
    m_dCost = m_dCost + PARAM_COST_COMFORT * cost_comfort;

    // 5) EFFICIENCY cost
    cost_efficiency = PARAM_FOV - predict.GetLaneFreeSpace(target.lane);
    m_dCost = m_dCost + PARAM_COST_EFFICIENCY * cost_efficiency;

    cout << "car_lane=" << car_lane << " target.lane=" << target.lane << " target_lvel=" << predict.GetLaneSpeed(target.lane) << " cost=" << m_dCost << endl;
}
```



### Output

Finally, the trajectory with the lowest cost is selected as the actual trajectory which the ego car will drive according with.  

```c++
double min_cost = trajectoryGenerator.getMinCost();
int min_cost_index = trajectoryGenerator.getMinCostIndex();
vector<double> next_x_vals = trajectoryGenerator.getMinCostTrajectoryXY().x_vals;
vector<double> next_y_vals = trajectoryGenerator.getMinCostTrajectoryXY().y_vals;

......

json msgJson;

msgJson["next_x"] = next_x_vals;
msgJson["next_y"] = next_y_vals;

auto msg = "42[\"control\","+ msgJson.dump()+"]";

ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
```
