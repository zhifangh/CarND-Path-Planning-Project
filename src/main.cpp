#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

#include <math.h>
#include "params.h"

#include "map.h"

#include "PredictionModel.h"
#include "TrajectoryGenerator.h"
#include "BehaviorPlanner.h"
#include "CostEvaluation.h"

using namespace std;


int main() {
    uWS::Hub h;

    // Load map
    Map map("../data/highway_map.csv");

    EgoVehicle car(0., 0., 0., 0., 0.,  0., 1.0, 0., false);

    bool start = true;
    
    TrajectorySD previousTrajectorySD;

    h.onMessage([&map, &car, &start, &previousTrajectorySD]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(data);

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    TrajectoryXY previousTrajectoryXY;

                    // Main car's localization Data
                    car.x = j[1]["x"];          // ["x"] The car's x position in map coordinates
                    car.y = j[1]["y"];          // ["y"] The car's y position in map coordinates
                    car.s = j[1]["s"];          // ["s"] The car's s position in frenet coordinates
                    car.d = j[1]["d"];          // ["d"] The car's d position in frenet coordinates
                    car.yaw = j[1]["yaw"];      // ["yaw"] The car's yaw angle in the map
                    car.speed = j[1]["speed"];  // ["speed"] The car's speed in MPH

                    // Previous path data given to the Planner
                    vector<double> previous_path_x = j[1]["previous_path_x"]; // ["previous_path_x"] The previous list of x points previously given to the simulator
                    vector<double> previous_path_y = j[1]["previous_path_y"]; // ["previous_path_y"] The previous list of y points previously given to the simulator

                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"]; // ["end_path_s"] The previous list's last point's frenet s value
                    double end_path_d = j[1]["end_path_d"]; // ["end_path_d"] The previous list's last point's frenet d value

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    /*
                     * A 2d vector of cars and that car's attributes.
                     * car's unique ID,
                     * car's x position in map coordinates,
                     * car's y position in map coordinates,
                     * car's x velocity in m/s,
                     * car's y velocity in m/s,
                     * car's s position in frenet coordinates,
                     * car's d position in frenet coordinates.
                    */
                    vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

                    previousTrajectoryXY.x_vals = previous_path_x;
                    previousTrajectoryXY.y_vals = previous_path_y;

                    vector<double> frenet_car = map.getFrenet(car.x, car.y, deg2rad(car.yaw));
                    car.s = frenet_car[0];
                    car.d = frenet_car[1];
                    car.lane = GetLaneIndex(car.d);

                    if (start)
                    {
                        TrajectoryJMT trajectoryJMT = JMT_init(car.s, car.d);
                        previousTrajectorySD = trajectoryJMT.path_sd;
                        start = false;
                    }

                    int prev_size = previousTrajectoryXY.x_vals.size();
                    // keep prev_size points from previous generated trajectory
                    // ohter points will be re-generated
                    PreviousPath previousPath = PreviousPath(previousTrajectoryXY, previousTrajectorySD, min(prev_size, PARAM_PREV_PATH_XY_REUSED));

                    // 50 points  <--> 1 second horizon
                    PredictionModel predictionModel = PredictionModel(sensor_fusion, car, PARAM_NB_POINTS);

                    BehaviorPlanner behaviorPlanner = BehaviorPlanner(sensor_fusion, car, predictionModel);
                    vector<TargetConfig> vTargetConfig = behaviorPlanner.GetTargetConfig();

                    TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator(vTargetConfig, map, car, previousPath, predictionModel);

                    double min_cost = trajectoryGenerator.getMinCost();
                    int min_cost_index = trajectoryGenerator.getMinCostIndex();
                    vector<double> next_x_vals = trajectoryGenerator.getMinCostTrajectoryXY().x_vals;
                    vector<double> next_y_vals = trajectoryGenerator.getMinCostTrajectoryXY().y_vals;

                    if (PARAM_TRAJECTORY_JMT)
                    {
                        previousTrajectorySD = trajectoryGenerator.getMinCostTrajectorySD();
                    }

                    int target_lane = vTargetConfig[min_cost_index].lane;
                    car.speed_target = vTargetConfig[min_cost_index].velocity;

                    if (target_lane != car.lane)
                    {
                      cout << "====================> CHANGE LANE: lowest cost for target " << min_cost_index << " = (target_lane=" << target_lane
                           << " target_vel=" << car.speed_target << " car.lane=" << car.lane << " cost="<< min_cost << ")" << endl;
                    }

                    json msgJson;

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                      char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}
