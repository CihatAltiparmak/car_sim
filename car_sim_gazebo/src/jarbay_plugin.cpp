/*
 * MIT License
 *
 * Copyright (c) 2022 CihatAltiparmak
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <car_sim_gazebo/jarbay_plugin.hpp>

#include <iostream>
namespace gazebo {

JarbayPlugin::JarbayPlugin() {
    target_steering_angle = 0.0;
    target_speed = 0.0;

    left_steering_pid  = common::PID(2e3, 0.0, 3e2);
    left_steering_pid.SetCmdMin(-5000.0);
    left_steering_pid.SetCmdMax(5000.0);

    right_steering_pid = common::PID(2e3, 0.0, 3e2);
    right_steering_pid.SetCmdMin(-5000.0);
    right_steering_pid.SetCmdMax(5000.0);

    wheel_front_left_pid  = common::PID(1000, 0.0, 1.0);
    wheel_front_right_pid = common::PID(1000, 0.0, 1.0);
    wheel_rear_left_pid   = common::PID(1000, 0.0, 1.0);
    wheel_rear_right_pid  = common::PID(1000, 0.0, 1.0);
}

JarbayPlugin::~JarbayPlugin() {
}

void JarbayPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    // Gazebo initialization
    steer_fl_joint = model->GetJoint("steer_fl_joint");
    steer_fr_joint = model->GetJoint("steer_fr_joint");
    wheel_rl_joint = model->GetJoint("wheel_rl_joint");
    wheel_rr_joint = model->GetJoint("wheel_rr_joint");
    wheel_fl_joint = model->GetJoint("wheel_fl_joint");
    wheel_fr_joint = model->GetJoint("wheel_fr_joint");

    footprint_link = model->GetLink("base_footprint");

    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&JarbayPlugin::onUpdate, this, _1));

    // ROS initialization
    ros_node = gazebo_ros::Node::Get(sdf);

    ros_node->declare_parameter<std::string>("drive_command_topic", "/itusct/command_cmd");
    ros_node->get_parameter("drive_command_topic", drive_command_topic);

    drive_sub = ros_node->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(drive_command_topic, 
        1, std::bind(&JarbayPlugin::onDrive, this, std::placeholders::_1));

    std::cout << "drive command topic: " << drive_command_topic << std::endl;

}

void JarbayPlugin::onDrive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive_msg) {
    target_speed          = drive_msg->drive.speed;
    target_steering_angle = drive_msg->drive.steering_angle;

    if (target_speed > MAX_SPEED) {
        target_speed = MAX_SPEED;
    }

    if (target_speed < -MAX_SPEED) {
        target_speed = -MAX_SPEED;
    }

    if (target_steering_angle > MAX_STEERING_ANGLE) {
        target_steering_angle = MAX_STEERING_ANGLE;
    }

    if (target_steering_angle < -MAX_STEERING_ANGLE) {
        target_steering_angle = -MAX_STEERING_ANGLE;
    }
}

void JarbayPlugin::onUpdate(const common::UpdateInfo& info) {
    if (last_update_time == common::Time(0)) {
        last_update_time = info.simTime;
        return;
    }

    double dt = (info.simTime - last_update_time).Double();
    last_update_time = info.simTime;

    updateSteering(dt);
    updateSpeed(dt);
}

void JarbayPlugin::updateSteering(double dt) {

    auto current_fl_steering_angle = steer_fl_joint->Position(0);
    auto current_fr_steering_angle = steer_fr_joint->Position(0);

    double t_alph = tan(target_steering_angle);
    double target_fr_steering_angle = atan(WHEELBASE * t_alph / (WHEELBASE + 0.5 * TRACK_WIDTH * t_alph));;
    double target_fl_steering_angle = atan(WHEELBASE * t_alph / (WHEELBASE - 0.5 * TRACK_WIDTH * t_alph));;

    double fr_steering_angle_error = current_fr_steering_angle - target_fr_steering_angle;
    double fr_steering_force = right_steering_pid.Update(fr_steering_angle_error, dt);
    steer_fr_joint->SetForce(0, fr_steering_force);

    double fl_steering_angle_error = current_fl_steering_angle - target_fl_steering_angle;
    double fl_steering_force = left_steering_pid.Update(fl_steering_angle_error, dt);
    steer_fl_joint->SetForce(0, fl_steering_force);
}

void JarbayPlugin::updateSpeed(double dt) {
    double wheel_rl_velocity = wheel_rl_joint->GetVelocity(0);
    double wheel_rl_velocity_error = wheel_rl_velocity - target_speed;
    double wheel_rl_velocity_force = wheel_rear_left_pid.Update(wheel_rl_velocity_error, dt);
    wheel_rl_joint->SetForce(0, wheel_rl_velocity_force);

    double wheel_rr_velocity = wheel_rr_joint->GetVelocity(0);
    double wheel_rr_velocity_error = wheel_rr_velocity - target_speed;
    double wheel_rr_velocity_force = wheel_rear_right_pid.Update(wheel_rr_velocity_error, dt);
    wheel_rr_joint->SetForce(0, wheel_rr_velocity_force);
}

void JarbayPlugin::Reset() {
}

} // end of namespace gazebo