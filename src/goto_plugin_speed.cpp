/*!*******************************************************************************************
 *  \file       goto_plugin_speed.cpp
 *  \brief      This file contains the implementation of the go to Behaviour speed plugin
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "goto_base.hpp"
#include "motion_reference_handlers/speed_motion.hpp"

namespace goto_plugins
{
    class GotoSpeed : public goto_base::GotoBase
    {
    public:
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) override
        {
            desired_position_ = Eigen::Vector3d(goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
            if (goal->max_speed != 0.0f)
            {
                desired_speed_ = goal->max_speed;
            }
            ignore_yaw_ = goal->ignore_pose_yaw;
            distance_measured_ = false;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleGoto> goal_handle) override
        {
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        bool onExecute(const std::shared_ptr<GoalHandleGoto> goal_handle) override
        {
            rclcpp::Rate loop_rate(10);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<as2_msgs::action::GoToWaypoint::Feedback>();
            auto result = std::make_shared<as2_msgs::action::GoToWaypoint::Result>();

            static as2::motionReferenceHandlers::SpeedMotion motion_handler(node_ptr_);

            float desired_yaw = ignore_yaw_ ? getActualYaw() : 0.0;
            RCLCPP_INFO(node_ptr_->get_logger(), "Desired yaw set to %f", desired_yaw);

            // Check if goal is done
            while (!checkGoalCondition())
            {
                if (goal_handle->is_canceling())
                {
                    result->goto_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_WARN(node_ptr_->get_logger(), "Goal canceled");
                    // TODO: change this to hover
                    motion_handler.sendSpeedCommandWithYawSpeed(0, 0, 0, 0);
                    return false;
                }

                pose_mutex_.lock();
                Eigen::Vector3d speed_setpoint = (desired_position_ - actual_position_);
                pose_mutex_.unlock();

                float yaw = ignore_yaw_ ? 0.0 : -atan2f((double)speed_setpoint.x(), (double)speed_setpoint.y()) + M_PI / 2.0f;
                motion_handler.sendSpeedCommandWithYawAngle(getValidSpeed(speed_setpoint.x()),
                                                            getValidSpeed(speed_setpoint.y()),
                                                            getValidSpeed(speed_setpoint.z()),
                                                            getValidSpeed(yaw));

                feedback->actual_distance_to_goal = actual_distance_to_goal_;
                feedback->actual_speed = actual_speed_;
                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            result->goto_success = true;
            goal_handle->succeed(result);
            // TODO: change this to hover
            motion_handler.sendSpeedCommandWithYawSpeed(0, 0, 0, 0);
            return true;
        }

    private:
        float getActualYaw()
        {
            pose_mutex_.lock();
            Eigen::Matrix3d rot_mat = actual_q_.toRotationMatrix();
            pose_mutex_.unlock();
            Eigen::Vector3d orientation = rot_mat.eulerAngles(0, 1, 2);
            float yaw = orientation[2] + M_PI / 2.0f;
            return yaw;
        }

        bool checkGoalCondition()
        {
            if (distance_measured_)
            {
                if (fabs(actual_distance_to_goal_) < goal_threshold_)
                    return true;
            }
            return false;
        }

        float getValidSpeed(float speed)
        {
            if (std::abs(speed) > desired_speed_)
            {
                return (speed < 0.0) ? -desired_speed_ : desired_speed_;
            }
            return speed;
        }

    private:
    }; // GotoSpeed class
} // goto_plugins namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(goto_plugins::GotoSpeed, goto_base::GotoBase)
