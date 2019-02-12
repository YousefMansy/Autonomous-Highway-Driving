[//]: # (Image References)

[image1]: ./pictures/left_turn_1.gif "left_turn_1"
[image2]: ./pictures/right_turn_1.gif "right_turn_1"
[image3]: ./pictures/right_turn_2.gif "right_turn_2"

# Autonomous Highway Driving
The aim of this project is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

![alt text][image3] ![alt text][image2]

### Description
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car avoids hitting other cars at all cost as well as makes sure to drive inside of the marked road lanes at all times, unless going from one lane to another. The car is able to make a complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car doesn't experience total acceleration over 10 m/s^2 and jerk over than 10 m/s^3.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Simulator.
You can download the Simulator used in this project from [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Planner Model Documentation

Each time increment (0.2 sec), the `get_next` function in the planner is called to generate a new trajectory path.

What the planner does first is go through the `sensor_fusion` data structure, which contains information about surrounding vehicles. The planner searches for any cars in the same lane as ours, and calculates the gap between our car and that car. If the gap is too low, a flag `too_close` is raised, to alert for lowering speed.

```
// Check gap size
if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
{
    // Flag for lowering reference velocity to avoid crashing and ask for a lange change
    too_close = true;
```

Moreover, if the gap isn't too low, another flag `change_lane` is raised, to alert for attempting a lane change.

```
// Don't change lanes if the car in front is way too close, in case a car suddenly switches to our lane
if ((check_car_s - car_s) > 15)
    change_lane = true;
```

Speed is then decreased at small increments (small enough to avoid high acceleration) every time cycle, until the `too_close` flag is no longer up. 

```
if (too_close)
    ref_velocity -= max_acceleration;
```

When, the `too_close` flag is no longer up, speed is increased again at small increments, until the speed limit is reached.

```
else if (ref_velocity < max_velocity - max_acceleration) // If no obstacles ahead and we're under speed limit, speed up
        ref_velocity += max_acceleration;
```

If the `change_lane` flag is up, a lane change is attempted. The planner checks if a lane transition is possible by calculating the s gap between our car and cars in neighboring lanes, and only switches if the cars in the target lane aren't too close.

```
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
```

Next, the planner starts generating a list of widely spaced (x,y) waypoints, spaced evenly at 30m. The starting reference is created so that it is tangent to the last two points in the previous path of the vehicle.

```
ref_x = previous_path_x[prev_size - 1];
ref_y = previous_path_y[prev_size - 1];

double ref_x_prev = previous_path_x[prev_size - 2];
double ref_y_prev = previous_path_y[prev_size - 2];
ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

ptsx.push_back(ref_x_prev);
ptsx.push_back(ref_x);

ptsy.push_back(ref_y_prev);
ptsy.push_back(ref_y);
```

In case the previous path is empty (the car is just starting), the car position itself is used as reference instead.

The rest of the path is populated using 30m spaced points (In Frenet).

```
vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

```



Now we need to smooth out this path. To do that, we use the spline library. 

```
tk::spline s;
```

We pass our current path to the spline library, and it generates a similar smoothed out path from it.

```
s.set_points(ptsx, ptsy);
```

However, to maintain our reference speed, we need to figure out how to space out the spline points appropriately. In other words, we need to calculate `N`, where `N` represents the number of pieces the spline will be split into. To do that, can use the formula:
```
N * 0.2 * velocity = distance
```
Where 0.2 is the time cycle frequency, velocity is our reference velocity, and distance is the distance between the car and the target point. Here, we calculate that distance
```
double target_x = 30.0;
double target_y = s(target_x); // Where the target lies on the spline
double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
```
Then we use it to calculate N
```
double N = (target_dist / (.02 * ref_velocity / 2.24));
```

Finally we fill the path with our spline points, spaced out at the calculated `N`.




