#include "goto_base.hpp"
#include "as2_motion_command_handlers/speed_motion.hpp"

namespace goto_plugins
{
    class GotoSpeed : public goto_base::GotoBase
    {
    public:
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) override
        {
            desired_position_ = Eigen::Vector3d(goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
            desired_speed_ = goal->max_speed;
            ignore_yaw_ = goal->ignore_pose_yaw;
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

            static as2::motionCommandsHandlers::SpeedMotion motion_handler(node_ptr_);

            float desired_yaw = ignore_yaw_ ? getActualYaw() : 0.0;
            RCLCPP_INFO(node_ptr_->get_logger(), "Desired yaw set to %f", desired_yaw);

            // Check if goal is done
            while (!checkGoalCondition())
            {
                if (goal_handle->is_canceling())
                {
                    result->goto_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
                    // TODO: change this to hover
                    motion_handler.sendSpeedCommandWithYawSpeed(0, 0, 0, 0);
                    return false;
                }

                pose_mutex_.lock();
                Eigen::Vector3d speed_setpoint = (desired_position_ - actual_position_);
                pose_mutex_.unlock();

                float vyaw = ignore_yaw_ ? 0.0 : -atan2f((double)speed_setpoint.x(), (double)speed_setpoint.y()) + M_PI / 2.0f;
                motion_handler.sendSpeedCommandWithYawSpeed(getValidSpeed(speed_setpoint.x()),
                                                            getValidSpeed(speed_setpoint.y()),
                                                            getValidSpeed(speed_setpoint.z()), 
                                                            getValidSpeed(vyaw));

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