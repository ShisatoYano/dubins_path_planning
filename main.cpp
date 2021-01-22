#include <bits/stdc++.h>

using namespace std;

float deg2rad(float deg)
{
    return (deg * M_PI / 180.0);
}

int main()
{
    // start state
    float start_x = 1.0; // [m]
    float start_y = 1.0; // [m]
    float start_yaw = deg2rad(45.0); // [rad]

    // goal state
    float goal_x = -3.0; // [m]
    float goal_y = -3.0; // [m]
    float goal_yaw = deg2rad(-45.0); // [rad]

    float curvature = 1.0;

    return 0;
}
