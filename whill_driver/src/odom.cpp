/*
MIT License
Copyright (c) 2018-2019 WHILL inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <memory>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "whill_driver/odom.h"

const float base_link_height = 0.1325;

Odometry::Odometry()
{
    pose.x = pose.y = pose.theta = 0.0;
    velocity.x = velocity.y = velocity.theta = 0.0;
    // debug 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Odometry constructor");
}

long double Odometry::confineRadian(long double rad)
{
    if (rad >= M_PI)
    {
        rad -= 2.0 * M_PI;
    }
    if (rad <= -M_PI)
    {
        rad += 2.0 * M_PI;
    }
    return rad;
}

void Odometry::setParameters(double _wheel_radius, double _wheel_tread)
{
    this->wheel_radius = _wheel_radius;
    this->wheel_tread = _wheel_tread;
}

void Odometry::update(sensor_msgs::msg::JointState joint_state, double dt)
{
    if (dt <= 0.01 || std::isnan(dt) || std::isinf(dt))  // dtの確認と無効な値の処理
    {
        return;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint state velocity size: %zu", joint_state.velocity.size());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint state position size: %zu", joint_state.position.size());

    double angle_vel_r = joint_state.velocity[1];
    double angle_vel_l = -joint_state.velocity[0];

    // 車輪の回転速度を平滑化
    double filter_factor = 0.1;  // フィルター係数
    static double smoothed_vr = 0.0;
    static double smoothed_vl = 0.0;

    smoothed_vr = (1 - filter_factor) * smoothed_vr + filter_factor * angle_vel_r;
    smoothed_vl = (1 - filter_factor) * smoothed_vl + filter_factor * angle_vel_l;

    // 車輪の速度
    double vr = smoothed_vr * 0.1325;  // 速度に車輪の半径を掛ける
    double vl = smoothed_vl * 0.1325;

    // 進行距離と角速度の計算
    double delta_L = (vr + vl) / 2.0;
    double delta_theta = (vr - vl) / 0.496;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dt: %f", dt);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "delta_L: %f, delta_theta: %f", delta_L, delta_theta);

    // 現在のオドメトリに基づいて位置を更新
    pose.x += delta_L * dt * std::cos(pose.theta + delta_theta * dt / 2.0);
    pose.y += delta_L * dt * std::sin(pose.theta + delta_theta * dt / 2.0);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pose x: %f, y: %f, theta: %f", pose.x, pose.y, pose.theta);

    // 速度の更新
    velocity.x = delta_L;
    velocity.y = 0.0;
    velocity.theta = delta_theta;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "velocity x: %f, y: %f, theta: %f", velocity.x, velocity.y, velocity.theta);

    // 角度の制限
    double max_delta_theta = 0.1;  // 最大角度変化を制限
    delta_theta = std::clamp(delta_theta, -max_delta_theta, max_delta_theta);
    
    double theta = pose.theta + delta_theta * dt;
    pose.theta = confineRadian(theta);

    return;
}

void Odometry::zeroVelocity()
{
    velocity.x = 0;
    velocity.y = 0;
    velocity.theta = 0;
    return;
}

void Odometry::reset()
{
    Space2D poseZero = {0, 0, 0};
    set(poseZero);
    velocity = poseZero;
}

void Odometry::set(Space2D pose)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set pose x: %f, y: %f, theta: %f", pose.x, pose.y, pose.theta);
    this->pose = pose;
}

Odometry::Space2D Odometry::getOdom()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "getOdom pose x: %f, y: %f, theta: %f", pose.x, pose.y, pose.theta);
    return pose;
}

nav_msgs::msg::Odometry Odometry::getROSOdometry()
{
    nav_msgs::msg::Odometry odom;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, pose.theta);

    // 位置の設定
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = base_link_height;
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();

    // 速度の設定
    odom.twist.twist.linear.x = velocity.x;
    odom.twist.twist.linear.y = velocity.y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = velocity.theta;

    //debug
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pose x: %f, y: %f, theta: %f", pose.x, pose.y, pose.theta);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "odom x: %f, y: %f, theta: %f", pose.x, pose.y, pose.theta);

    return odom;
}

geometry_msgs::msg::TransformStamped Odometry::getROSTransformStamped()
{
    geometry_msgs::msg::TransformStamped odom_trans;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, pose.theta);

    odom_trans.header.stamp = rclcpp::Clock().now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose.x;
    odom_trans.transform.translation.y = pose.y;
    odom_trans.transform.translation.z = base_link_height;
    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();

    return odom_trans;
}
