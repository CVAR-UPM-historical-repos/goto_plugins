#include "goto_base.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/srv/send_trajectory_waypoints.hpp"
#include "as2_msgs/srv/set_speed.hpp"
#include "as2_core/names/services.hpp"

#include "as2_core/synchronous_service_client.hpp"

namespace goto_plugins
{
    class GotoTraj : public goto_base::GotoBase
    {
        using YAW_MODE = as2_msgs::msg::TrajectoryWaypointsWithID;
        using SyncSetSpeed = as2::SynchronousServiceClient<as2_msgs::srv::SetSpeed>;
        using SyncSendTrajWayp = as2::SynchronousServiceClient<as2_msgs::srv::SendTrajectoryWaypoints>;
    public:
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) override
        {
            desired_position_ = Eigen::Vector3d(goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
            if (goal->max_speed != 0.0f)
            {
                desired_speed_ = goal->max_speed;
            }

            auto req_speed = as2_msgs::srv::SetSpeed::Request();
            auto resp_speed = as2_msgs::srv::SetSpeed::Response();

            req_speed.speed.speed = desired_speed_;

            auto set_traj_speed_cli = SyncSetSpeed(as2_names::services::motion_reference::set_traj_speed);
            if (!set_traj_speed_cli.sendRequest(req_speed, resp_speed, 1))
            {
                return rclcpp_action::GoalResponse::REJECT;
            }

            auto req_traj = as2_msgs::srv::SendTrajectoryWaypoints::Request();
            auto resp_traj = as2_msgs::srv::SendTrajectoryWaypoints::Response();
            as2_msgs::msg::PoseStampedWithID pose;
            // pose.id = "goto_0";
            pose.pose = goal->target_pose;
            req_traj.waypoints.poses.emplace_back(pose);
            req_traj.waypoints.yaw_mode = goal->ignore_pose_yaw ? YAW_MODE::KEEP_YAW : YAW_MODE::PATH_FACING;

            auto send_traj_wayp_cli = SyncSendTrajWayp(as2_names::services::motion_reference::send_traj_wayp);
            if (!send_traj_wayp_cli.sendRequest(req_traj, resp_traj, 1))
            {
                return rclcpp_action::GoalResponse::REJECT;
            }

            distance_measured_ = false;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleGoto> goal_handle) override
        {
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        bool onExecute(const std::shared_ptr<GoalHandleGoto> goal_handle) override
        {
            auto feedback = std::make_shared<as2_msgs::action::GoToWaypoint::Feedback>();
            auto result = std::make_shared<as2_msgs::action::GoToWaypoint::Result>();

            rclcpp::Rate loop_rate(10);
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
    }; // GotoTraj class
} // goto_plugins namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(goto_plugins::GotoTraj, goto_base::GotoBase)