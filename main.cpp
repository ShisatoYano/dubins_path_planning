#include <bits/stdc++.h>

using namespace std;

float deg2rad(float deg)
{
    return (deg * M_PI / 180.0);
}

float rad2deg(float rad)
{
    return (rad * 180.0 / M_PI);
}

float mod2pi(float theta)
{
    return (theta - 2.0 * M_PI * floor(theta / 2.0 / M_PI));
}

float pi_2_pi(float angle)
{
    if (angle >= M_PI) {angle -= 2.0 * M_PI;}

    if (angle <= -M_PI) {angle += 2.0 * M_PI;}

    return angle;
}

class LSL
{
private:
public:
    float t, p, q;
    string mode = "LSL";

    LSL(float alpha, float beta, float d)
    {
        float sa = sin(alpha);
        float sb = sin(beta);
        float ca = cos(alpha);
        float cb = cos(beta);
        float c_ab = cos(alpha - beta);
        float tmp0 = d + sa - sb;

        float p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));
        if (p_squared < 0)
        {
            t = 0.0; p = 0.0; q = 0.0;
        }
        float tmp1 = atan2((cb - ca), tmp0);
        t = mod2pi(-alpha + tmp1);
        p = sqrt(p_squared);
        q = mod2pi(beta - tmp1);
    }
};

class RSR
{
private:
public:
    float t, p, q;
    string mode = "RSR";

    RSR(float alpha, float beta, float d)
    {
        float sa = sin(alpha);
        float sb = sin(beta);
        float ca = cos(alpha);
        float cb = cos(beta);
        float c_ab = cos(alpha - beta);
        float tmp0 = d - sa + sb;

        float p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa));
        if (p_squared < 0)
        {
            t = 0.0; p = 0.0; q = 0.0;
        }
        float tmp1 = atan2((ca - cb), tmp0);
        t = mod2pi(alpha - tmp1);
        p = sqrt(p_squared);
        q = mod2pi(-beta + tmp1);
    }
};

class LSR
{
private:
public:
    float t, p, q;
    string mode = "LSR";

    LSR(float alpha, float beta, float d)
    {
        float sa = sin(alpha);
        float sb = sin(beta);
        float ca = cos(alpha);
        float cb = cos(beta);
        float c_ab = cos(alpha - beta);

        float p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb));
        if (p_squared < 0)
        {
            t = 0.0; p = 0.0; q = 0.0;
        }
        p = sqrt(p_squared);
        float tmp2 = atan2((-ca - cb), (d + sa + sb)) - atan2(-2.0, p);
        t = mod2pi(-alpha + tmp2);
        q = mod2pi(-mod2pi(beta) + tmp2);
    }
};

class RSL
{
private:
public:
    float t, p, q;
    string mode = "RSL";

    RSL(float alpha, float beta, float d)
    {
        float sa = sin(alpha);
        float sb = sin(beta);
        float ca = cos(alpha);
        float cb = cos(beta);
        float c_ab = cos(alpha - beta);

        float p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb));
        if (p_squared < 0)
        {
            t = 0.0; p = 0.0; q = 0.0;
        }
        p = sqrt(p_squared);
        float tmp2 = atan2((ca + cb), (d - sa - sb)) - atan2(2.0, p);
        t = mod2pi(alpha - tmp2);
        q = mod2pi(beta - tmp2);
    }
};

class RLR
{
private:
public:
    float t, p, q;
    string mode = "RLR";

    RLR(float alpha, float beta, float d)
    {
        float sa = sin(alpha);
        float sb = sin(beta);
        float ca = cos(alpha);
        float cb = cos(beta);
        float c_ab = cos(alpha - beta);

        float tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
        if (abs(tmp_rlr) > 1.0)
        {
            t = 0.0; p = 0.0; q = 0.0;
        }
        p = mod2pi(2 * M_PI - acos(tmp_rlr));
        t = mod2pi(alpha - atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0));
        q = mod2pi(alpha - beta - t + mod2pi(p));
    }
};

class LRL
{
private:
public:
    float t, p, q;
    string mode = "LRL";

    LRL(float alpha, float beta, float d)
    {
        float sa = sin(alpha);
        float sb = sin(beta);
        float ca = cos(alpha);
        float cb = cos(beta);
        float c_ab = cos(alpha - beta);

        float tmp_lrl = (6.0 - d * d + 2 * c_ab + 2 * d * (-sa + sb)) / 8.0;
        if (abs(tmp_lrl) > 1)
        {
            t = 0.0; p = 0.0; q = 0.0;
        }
        p = mod2pi(2 * M_PI - acos(tmp_lrl));
        t = mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + p / 2.0);
        q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));
    }
};

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
    vector<float> px, py, pyaw;
    vector<float> ox, oy, oyaw;
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
        // normalize
        float dx = lgx;
        float dy = lgy;
        float D = sqrt(dx * dx + dy * dy);
        float d = D / curvature;

        float theta = mod2pi(atan2(dy, dx));
        float alpha = mod2pi(-theta);
        float beta = mod2pi(lgyaw - theta);

        // initialize cost
        float bcost = INFINITY;
        float cost;
        float bt = 0.0, bp = 0.0, bq = 0.0;
        string bmode = "NUL";

        // planners
        LSL lsl = LSL(alpha, beta, d);
        if (lsl.t != 0.0)
        {
            cost = abs(lsl.t) + abs(lsl.p) + abs(lsl.q);
            if (bcost > cost)
            {
                bt = lsl.t, bp = lsl.p, bq = lsl.q, bmode = lsl.mode;
                bcost = cost;
            }
        }

        RSR rsr = RSR(alpha, beta, d);
        if (rsr.t != 0.0)
        {
            cost = abs(rsr.t) + abs(rsr.p) + abs(rsr.q);
            if (bcost > cost)
            {
                bt = rsr.t, bp = rsr.p, bq = rsr.q, bmode = rsr.mode;
                bcost = cost;
            }
        }

        LSR lsr = LSR(alpha, beta, d);
        if (lsr.t != 0.0)
        {
            cost = abs(lsr.t) + abs(lsr.p) + abs(lsr.q);
            if (bcost > cost)
            {
                bt = lsr.t, bp = lsr.p, bq = lsr.q, bmode = lsr.mode;
                bcost = cost;
            }
        }

        RSL rsl = RSL(alpha, beta, d);
        if (rsl.t != 0.0)
        {
            cost = abs(rsl.t) + abs(rsl.p) + abs(rsl.q);
            if (bcost > cost)
            {
                bt = rsl.t, bp = rsl.p, bq = rsl.q, bmode = rsl.mode;
                bcost = cost;
            }
        }

        RLR rlr = RLR(alpha, beta, d);
        if (rlr.t != 0.0)
        {
            cost = abs(rlr.t) + abs(rlr.p) + abs(rlr.q);
            if (bcost > cost)
            {
                bt = rlr.t, bp = rlr.p, bq = rlr.q, bmode = rlr.mode;
                bcost = cost;
            }
        }

        LRL lrl = LRL(alpha, beta, d);
        if (lrl.t != 0.0)
        {
            cost = abs(lrl.t) + abs(lrl.p) + abs(lrl.q);
            if (bcost > cost)
            {
                bt = lrl.t, bp = lrl.p, bq = lrl.q, bmode = lrl.mode;
                bcost = cost;
            }
        }

        // generate course
        float length[3] = {bt, bp, bq};
        generate_course(length, bmode, curvature);
    }

    void generate_course(float length[], string bmode, float c)
    {
        float pd, d;

        px.push_back(0.0);
        py.push_back(0.0);
        pyaw.push_back(0.0);

        for (int i = 0; i < 3; ++i) {
            pd = 0.0;
            if (bmode[i] == 'S') {d = 1.0 / c;}
            else {d = deg2rad(3.0);}

            while (pd < abs(length[i] - d))
            {
                px.push_back(px.back() + d * c * cos(pyaw.back()));
                py.push_back(py.back() + d * c * sin(pyaw.back()));

                if (bmode[i] == 'L') {pyaw.push_back(pyaw.back() + d);}
                else if (bmode[i] == 'S') {pyaw.push_back(pyaw.back());}
                else if (bmode[i] == 'R') {pyaw.push_back(pyaw.back() - d);}
                pd += d;
            }
            d = length[i] - pd;
            px.push_back(px.back() + d * c * cos(pyaw.back()));
            py.push_back(py.back() + d * c * sin(pyaw.back()));

            if (bmode[i] == 'L') {pyaw.push_back(pyaw.back() + d);}
            else if (bmode[i] == 'S') {pyaw.push_back(pyaw.back());}
            else if (bmode[i] == 'R') {pyaw.push_back(pyaw.back() - d);}
            pd += d;
        }
    }

    void planning()
    {
        float lgx = cos(start_yaw) * (goal_x - start_x) +
                    sin(start_yaw) * (goal_y - start_y);
        float lgy = -sin(start_yaw) * (goal_x - start_x) +
                    cos(start_yaw) * (goal_y - start_y);
        float lgyaw = goal_yaw - start_yaw;

        planning_from_origin(lgx, lgy, lgyaw);

        // path output
        float x, y, yaw;
        for (int i = 0; i < px.size(); ++i) {
            x = cos(-start_yaw) * px[i] + sin(-start_yaw) * py[i] + start_x;
            y = -sin(-start_yaw) * px[i] + cos(-start_yaw) * py[i] + start_y;
            yaw = pi_2_pi(pyaw[i] + start_yaw);
            ox.push_back(x);
            oy.push_back(y);
            oyaw.push_back(yaw);
        }

        // print path
        for (int i = 0; i < ox.size(); ++i) {
            cout << "(" << ox[i] << ", " << oy[i] << ", " << rad2deg(oyaw[i]) << ")" << endl;
        }
    }
};

int main()
{
    DubinsPath dub = DubinsPath(1.0, 1.0, deg2rad(10.0),
                                -3.0, -3.0, deg2rad(-45.0),
                                1.0);

    dub.planning();

    return 0;
}
