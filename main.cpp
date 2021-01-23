#include <bits/stdc++.h>

using namespace std;

float deg2rad(float deg)
{
    return (deg * M_PI / 180.0);
}

class DubinsPath
{
private:

public:
    // inputs
    float start_x; // [m]
    float start_y; // [m]
    float start_yaw; // [rad]

    float goal_x; // [m]
    float goal_y; // [m]
    float goal_yaw; // [rad]

    float curvature;

    // output
    vector<float> ox;
    vector<float> oy;
    vector<float> oyaw;
    float ocost;
    string mode;

    DubinsPath(float sx, float sy, float syaw, float gx,
               float gy, float gyaw, float c)
    {
        start_x = sx; start_y = sy; start_yaw = syaw;
        goal_x = gx; goal_y = gy; goal_yaw = gyaw;
        curvature = c;
    }

    void planning_from_origin(float lgx, float lgy, float lgyaw)
    {

    }

    void planning()
    {
        float lgx = cos(start_yaw) * (goal_x - start_x) +
                    sin(start_yaw) * (goal_y - start_y);
        float lgy = -sin(start_yaw) * (goal_x - start_x) +
                    cos(start_yaw) * (goal_y - start_y);
        float lgyaw = goal_yaw - start_yaw;

        planning_from_origin(lgx, lgy, lgyaw);
    }
};

int main()
{
    DubinsPath dub = DubinsPath(1.0, 1.0, deg2rad(45.0),
                                -3.0, -3.0, deg2rad(-45.0),
                                1.0);

    dub.planning();

    return 0;
}
