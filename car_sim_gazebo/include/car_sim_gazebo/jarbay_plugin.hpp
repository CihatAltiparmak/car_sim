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

#ifndef _JARBAY_PLUGIN_HPP__
#define _JARBAY_PLUGIN_HPP__

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

double WHEELBASE          = 2.65;
double TRACK_WIDTH        = 1.638;
double MAX_SPEED          = 10;
double MAX_STEERING_ANGLE = 0.52;


namespace gazebo {

class JarbayPlugin : public ModelPlugin {
    public:
        JarbayPlugin();
        virtual ~JarbayPlugin();
    protected:
        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        virtual void Reset();
    private:
        void onUpdate(const common::UpdateInfo&);
        void onDrive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr);
        void updateSteering(double);
        void updateSpeed(double);
        
        event::ConnectionPtr update_connection;
        physics::JointPtr steer_fl_joint;
        physics::JointPtr steer_fr_joint;
        physics::JointPtr wheel_rl_joint;
        physics::JointPtr wheel_rr_joint;
        physics::JointPtr wheel_fl_joint;
        physics::JointPtr wheel_fr_joint;
        physics::LinkPtr footprint_link;
        
        gazebo_ros::Node::SharedPtr ros_node;
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub;
        
        common::PID left_steering_pid;
        common::PID right_steering_pid;
        
        common::PID wheel_rear_right_pid;
        common::PID wheel_rear_left_pid;
        
        common::PID wheel_front_right_pid;
        common::PID wheel_front_left_pid;

        double target_steering_angle;
        double target_speed;

        common::Time last_update_time;

        std::string drive_command_topic;
};

GZ_REGISTER_MODEL_PLUGIN(JarbayPlugin)
}

#endif