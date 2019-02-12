#include "planner.h"
#include "helpers.h"
#include "spline.h"

using namespace std;

Planner::Planner(double max_acceleration, double max_velocity, double ref_velocity, int lane, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                 vector<double> map_waypoints_dx, vector<double> map_waypoints_dy)
{
    this->max_acceleration = max_acceleration;
    this->max_velocity = max_velocity;
    this->ref_velocity = ref_velocity;
    this->lane = lane;
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_dx = map_waypoints_dx;
    this->map_waypoints_dy = map_waypoints_dy;
}

Planner::~Planner()
{
}

vector<vector<double>> Planner::getNext(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x,
                                        vector<double> previous_path_y, double end_path_s, double end_path_d, vector<vector<double>> sensor_fusion)
{
    int prev_size = previous_path_x.size();

    if (prev_size > 0)
    {
        car_s = end_path_s;
    }

    bool too_close = false;
    bool change_lane = false;
    // Find ref_v to use
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        // Check if the car is in my lane
        float d = sensor_fusion[i][6];
        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);

            double check_car_s = sensor_fusion[i][5];

            // If we're using old points, project the other car outwards a bit
            check_car_s += ((double)prev_size * .02 * check_speed);

            // Check gap size
            if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
            {
                // Flag for lowering reference velocity to avoid crashing and ask for a lange change
                too_close = true;

                // Don't change lanes if the car in front is way too close, in case a car suddenly switches to our lane
                if ((check_car_s - car_s) > 15)
                    change_lane = true;

                break;
            }
        }
    }

    if (too_close)
    {
        ref_velocity -= max_acceleration;
        if (change_lane)
        {
            if (car_d < (2 + 4 * 1 + 2) && car_d > (2 + 4 * 1 - 2)) // If current lane is 1
            {
                if (can_switch_lane(0, sensor_fusion, prev_size, car_s)) // Switch to lane 0 if possible
                    lane = 0;
                else if (can_switch_lane(2, sensor_fusion, prev_size, car_s)) // If not, try lane 2
                    lane = 2;
            }
            else if ((car_d < (2 + 4 * 0 + 2) && car_d > (2 + 4 * 0 - 2)) || (car_d < (2 + 4 * 2 + 2) && car_d > (2 + 4 * 2 - 2))) // If current lane is 0 or 2
            {
                if (can_switch_lane(1, sensor_fusion, prev_size, car_s)) // Switch to lane 1 if possible
                    lane = 1;
            }
        }
    }

    else if (ref_velocity < max_velocity - max_acceleration) // If no obstacles ahead and we're under speed limit, speed up
        ref_velocity += max_acceleration;

    return generate_path(car_x, car_y, car_yaw, prev_size, previous_path_x, previous_path_y, car_s);
}

bool Planner::can_switch_lane(int target_lane, vector<vector<double>> sensor_fusion, int prev_size, double car_s)
{
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float d = sensor_fusion[i][6];
        if (d < (2 + 4 * target_lane + 2) && d > (2 + 4 * target_lane - 2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);

            double check_car_s = sensor_fusion[i][5];

            // If we're using old points, project the other car outwards a bit
            check_car_s += ((double)prev_size * .02 * check_speed);

            // Check gap size
            if (abs(check_car_s - car_s) < 30)
                return false;
        }
    }
    return true;
}

vector<vector<double>> Planner::generate_path(double car_x, double car_y, double car_yaw, int prev_size, vector<double> previous_path_x,
                                              vector<double> previous_path_y, double car_s)
{
    // Create a list of widely space (x,y) waypoints, spaced evently at 30m
    vector<double> ptsx;
    vector<double> ptsy;

    // reference x, y, yaw states
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    // If previous path is empty, use the car as starting reference and create a tangent path
    if (prev_size < 2)
    {
        // Use two points that make the path tangent to the car
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    // Else, use previous path's end two points as starting reference, and create a tangent to that
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Add 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++)
    {
        // Shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // Create a spline
    tk::spline s;

    // Set (x,y) points to the spline
    s.set_points(ptsx, ptsy);

    // Define the actual (x,y) points we will use
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Start with all the remaining previous points
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Calculate how to break up spline points so that we travel at the desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

    double x_add_on = 0;

    // Fill up the rest of the path (50 points) with our new points
    for (int i = 1; i <= 50 - previous_path_x.size(); i++)
    {
        double N = (target_dist / (.02 * ref_velocity / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point; 

        // Rotate back to normal
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    vector<vector<double>> path;

    path.push_back(next_x_vals);
    path.push_back(next_y_vals);

    return path;
}