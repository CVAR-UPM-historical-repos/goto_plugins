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
#include "motion_reference_handlers/hover_motion.hpp"
#include "motion_reference_handlers/position_motion.hpp"

namespace goto_plugin_position
{
    class Plugin : public goto_base::GotoBase
    {
    private:
        float yaw_goal_;

    public:
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) override
        {
            desired_position_ = Eigen::Vector3d(goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
            if (goal->max_speed != 0.0f)
            {
                desired_speed_ = goal->max_speed;
            }
            ignore_yaw_ = goal->ignore_pose_yaw;

            // Calculate angle
            switch (goal->yaw_mode_flag)
            {
            case as2_msgs::action::GoToWaypoint::Goal::FIXED_YAW:
            {
                yaw_goal_ = as2::FrameUtils::getYawFromQuaternion(goal->target_pose.orientation);
                break;
            }
            case as2_msgs::action::GoToWaypoint::Goal::KEEP_YAW:
                yaw_goal_ = getActualYaw();
                break;
            case as2_msgs::action::GoToWaypoint::Goal::PATH_FACING:
                Eigen::Vector3d actual_position = getActualPosition();
                yaw_goal_ = computeFacingAngle(actual_position, desired_position_);
                break;
                return rclcpp_action::GoalResponse::REJECT;
            }

            // TODO: Use the yaw_mode_flag to set the yaw_goal_ when Python Interface supports it
            if (!ignore_yaw_)
            {
                RCLCPP_INFO(node_ptr_->get_logger(), "Path facing: %f", yaw_goal_);
                Eigen::Vector3d actual_position = getActualPosition();
                RCLCPP_INFO(node_ptr_->get_logger(), "Get actual position");
                yaw_goal_ = computeFacingAngle(actual_position, desired_position_);
            }

            RCLCPP_INFO(node_ptr_->get_logger(), "Goal Angle set to: %f", yaw_goal_);

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

            static as2::motionReferenceHandlers::PositionMotion motion_handler(node_ptr_);
            static as2::motionReferenceHandlers::HoverMotion motion_handler_hover(node_ptr_);

            // Check if goal is done
            while (!checkGoalCondition())
            {
                // TODO: Send only once not in the loop.
                motion_handler.sendPositionCommandWithYawAngle(desired_position_[0], desired_position_[1], desired_position_[2], yaw_goal_, desired_speed_, desired_speed_, desired_speed_);
                if (goal_handle->is_canceling())
                {

                    result->goto_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_WARN(node_ptr_->get_logger(), "Goal canceled");
                    motion_handler_hover.sendHover();
                    return false;
                }

                feedback->actual_distance_to_goal = actual_distance_to_goal_;
                feedback->actual_speed = actual_speed_;
                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            result->goto_success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
            // TODO: change this to hover?
            motion_handler.sendPositionCommandWithYawAngle(desired_position_[0], desired_position_[1], desired_position_[2], yaw_goal_, desired_speed_, desired_speed_, desired_speed_);
            return true;
        }

    private:
        float getValidSpeed(float speed)
        {
            if (std::abs(speed) > desired_speed_)
            {
                return (speed < 0.0) ? -desired_speed_ : desired_speed_;
            }
            return speed;
        }

        Eigen::Vector3d getActualPosition()
        {
            pose_mutex_.lock();
            Eigen::Vector3d position = actual_position_;
            pose_mutex_.unlock();
            return position;
        }

        double computeFacingAngle(Eigen::Vector3d fromPoint, Eigen::Vector3d toPoint)
        {
            Eigen::Vector3d diff = toPoint - fromPoint;
            if (diff.head(2).norm() < 2.0f)
            {
                return getActualYaw();
            }
            return as2::FrameUtils::getVector2DAngle(diff[0], diff[1]);
        }
    }; // Plugin class
} // goto_plugin_position namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(goto_plugin_position::Plugin, goto_base::GotoBase)
