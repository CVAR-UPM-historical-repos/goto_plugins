#include "goto_base.hpp"
#include "as2_msgs/msg/trajectory_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace goto_plugins
{
    class GotoTraj : public goto_base::GotoBase
    {
    public:
        void ownInit(as2::Node *node_ptr) override
        {
            traj_waypoints_pub_ = node_ptr_->create_publisher<as2_msgs::msg::TrajectoryWaypoints>(
                node_ptr_->generate_global_name(as2_names::topics::motion_reference::wayp), 
                as2_names::topics::motion_reference::qos_wp);
        }

        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) override
        {
            desired_position_ = Eigen::Vector3d(goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
            if (goal->max_speed != 0.0f) 
            {
                desired_speed_ = goal->max_speed;
            }

            as2_msgs::msg::TrajectoryWaypoints trajectory_waypoints;
            geometry_msgs::msg::PoseStamped pose;
            pose.pose = goal->target_pose;
            trajectory_waypoints.poses.push_back(pose);
            if (goal->ignore_pose_yaw) 
            {
                trajectory_waypoints.yaw_mode = as2_msgs::msg::TrajectoryWaypoints::KEEP_YAW;
            }
            else
            {
                trajectory_waypoints.yaw_mode = as2_msgs::msg::TrajectoryWaypoints::PATH_FACING;
            }
            trajectory_waypoints.max_speed = desired_speed_;
            traj_waypoints_pub_->publish(trajectory_waypoints);
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

            // Check if goal is done
            while (!checkGoalCondition())
            {
                if (goal_handle->is_canceling())
                {
                    result->goto_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_ptr_->get_logger(), "Goal can not be canceled");
                    return false;
                }

                feedback->actual_distance_to_goal = actual_distance_to_goal_;
                feedback->actual_speed = actual_speed_;
                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            result->goto_success = true;
            goal_handle->succeed(result);
            // TODO: change this to hover
            return true;
        }

    private:
        bool checkGoalCondition()
        {
            if (distance_measured_)
            {
                if (fabs(actual_distance_to_goal_) < goal_threshold_)
                    return true;
            }
            return false;
        }

    private:
        rclcpp::Publisher<as2_msgs::msg::TrajectoryWaypoints>::SharedPtr traj_waypoints_pub_;
    }; // GotoTraj class
} // goto_plugins namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(goto_plugins::GotoTraj, goto_base::GotoBase)