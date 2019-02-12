#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <vector>
#include <math.h>
#include <numeric>
#include <algorithm>

using namespace std;

class Planner
{
  public:
    Planner(double max_acceleration, double max_velocity, double ref_velocity, int lane, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
            vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
    virtual ~Planner();

    vector<vector<double>> getNext(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x,
                                   vector<double> previous_path_y, double end_path_s, double end_path_d, vector<vector<double>> sensor_fusion);

  private:
    double max_acceleration;
    double max_velocity;
    double ref_velocity;
    int lane;
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    bool can_switch_lane(int lane, vector<vector<double>> sensor_fusion, int prev_size, double car_s);
    vector<vector<double>> generate_path(double car_x, double car_y, double car_yaw, int prev_size, vector<double> previous_path_x, vector<double> previous_path_y, double car_s);
};

#endif //PLANNER_H