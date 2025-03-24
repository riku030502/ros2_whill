/*
MIT License
Copyright (c) 2018-2019 WHILL inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

// ros1
// #include "sensor_msgs/JointState.hpp"
// #include "nav_msgs/Odometry.hpp"
// #include "geometry_msgs/TransformStamped.hpp"

// ros2
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class Odometry
{
private:
    long double confineRadian(long double rad);

    typedef struct
    {
        // long double x;
        // long double y;
        // long double theta;
        double x;
        double y;
        double theta;
    } Space2D;

    double wheel_radius;
    double wheel_tread;

    Space2D pose;
    Space2D velocity;

public:
    Odometry();
    void setParameters(double _wheel_radius, double _wheel_tread);
    void update(sensor_msgs::msg::JointState joint, double dt);
    void zeroVelocity(void);
    void set(Space2D pose);
    void reset();

    nav_msgs::msg::Odometry getROSOdometry();
    geometry_msgs::msg::TransformStamped getROSTransformStamped();
    Space2D getOdom();
};