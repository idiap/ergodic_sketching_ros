// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <functional>

#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <ergodic_sketching_msgs/action/planner.hpp>
#include "ergodic_sketching_ros/Planner.hpp"

class PlannerActionServer: public rclcpp::Node{
public:
    using PlannerAction = ergodic_sketching_msgs::action::Planner;
    using GoalHandlePlanner = rclcpp_action::ServerGoalHandle<PlannerAction>;

    PlannerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("planner_action_server", options){
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<PlannerAction>(
            this,
            "plan",
            std::bind(&PlannerActionServer::handle_goal, this, _1, _2),
            std::bind(&PlannerActionServer::handle_cancel, this, _1),
            std::bind(&PlannerActionServer::handle_accepted, this, _1)
        );

        Planner::Configuration config;
        declare_parameter<std::string>("robot_description");
        declare_parameter<std::string>("base_frame");
        declare_parameter<std::string>("tip_frame");
        declare_parameter<int>("order");
        declare_parameter<int>("dof");
        declare_parameter<double>("rfactor");
        declare_parameter<int>("init_phase_length");
        declare_parameter<std::vector<double>>("q_max");
        declare_parameter<std::vector<double>>("q_min");
        declare_parameter<std::vector<double>>("dq_max");
        declare_parameter<std::vector<double>>("dq_min");
        declare_parameter<std::vector<double>>("q0");
        declare_parameter<std::vector<double>>("orn_thresh");
        declare_parameter<std::vector<std::string>>("joint_names");

        if (!get_parameter("robot_description", config.urdf)) {
            RCLCPP_ERROR(get_logger(),"No parameters robot_description");
            return;
        }

        if (!get_parameter("base_frame", config.base_frame)) {
            RCLCPP_ERROR(get_logger(),"No parameters base_frame");
            return;
        }

        if (!get_parameter("tip_frame", config.tip_frame)) {
            RCLCPP_ERROR(get_logger(),"No parameters tip_frame");
            return;
        }

        if (!get_parameter("order", config.order)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/order");
            return;
        }

        if (!get_parameter("dof", config.dof)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/dof");
            return;
        }

        if (!get_parameter("rfactor", config.rfactor)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/rfactor");
            return;
        }

        if (!get_parameter("init_phase_length", config.init_phase_length)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/init_phase_length");
            return;
        }

        config.nb_ctrl_var = config.dof;
        config.nb_state_var = 7;
        config.nb_iter = 25;
        config.dt = 0.01;
        config.batch_size = 7500;

        std::vector<double> q_max, q_min, dq_max, dq_min, q0, orn_thresh;

        if (!get_parameter("q_max", q_max)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/q_max");
            return;
        }
        config.q_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_max.data(), q_max.size());

        if (!get_parameter("q_min", q_min)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/q_min");
            return;
        }
        config.q_min = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_min.data(), q_min.size());

        if (!get_parameter("dq_max", dq_max)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/dq_max");
            return;
        }
        config.dq_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(dq_max.data(), dq_max.size());

        if (!get_parameter("dq_min", dq_min)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/dq_min");
            return;
        }
        config.dq_min = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(dq_min.data(), dq_min.size());

        if (!get_parameter("q0", q0)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/q0");
            return;
        }
        config.q0 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q0.data(), q0.size());

        if (!get_parameter("orn_thresh", orn_thresh)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/orn_thresh");
            return;
        }
        config.orn_thresh = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(orn_thresh.data(), orn_thresh.size());

        if (!get_parameter("joint_names", joint_names_)) {
            RCLCPP_ERROR(get_logger(),"No parameters ~/joint_names");
            return;
        }

        config.dq0 = Eigen::VectorXd::Zero(config.dof);
        planner_ = std::make_unique<Planner>(config);
    }
private:
    rclcpp_action::Server<PlannerAction>::SharedPtr action_server_;
    std::unique_ptr<Planner> planner_;
    std::vector<std::string> joint_names_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PlannerAction::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePlanner> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePlanner> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&PlannerActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlePlanner> goal_handle){
        std::shared_ptr<const PlannerAction::Goal> goal = goal_handle->get_goal();
        std::shared_ptr<PlannerAction::Result> result = std::make_shared<PlannerAction::Result>();

        std::vector<Eigen::Matrix<double, 7, 1>> states;
        std::vector<Eigen::Matrix<double, 6, 6>> precisions;

        for (auto const& pose : goal->path.poses) {
            Eigen::Matrix<double, 7, 1> state;
            state(0) = pose.pose.position.x;
            state(1) = pose.pose.position.y;
            state(2) = pose.pose.position.z;
            state(3) = pose.pose.orientation.w;
            state(4) = pose.pose.orientation.x;
            state(5) = pose.pose.orientation.y;
            state(6) = pose.pose.orientation.z;
            states.push_back(state);
        }

        Eigen::Matrix<double, 6, 6> precision = Eigen::Matrix<double, 6, 6>::Identity();
        precision(3, 3) = 0.01;
        precision(4, 4) = 0.01;
        precision(5, 5) = 0.01;

        precisions.resize(states.size(), precision);

        planner_->solve(states, precisions, [&](const std::vector<Eigen::VectorXd>& joint_positions, const std::vector<Eigen::VectorXd>& joint_velocities, const double& cost) {
            
            std::shared_ptr<PlannerAction::Feedback> feedback = std::make_shared<PlannerAction::Feedback>();
            feedback->traj.joint_names = joint_names_;
            feedback->traj.header.stamp = get_clock()->now();

            int i = 0;

            for (auto const& joint_position : joint_positions) {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = std::vector<double>(joint_position.data(), joint_position.data() + joint_position.rows());
                point.velocities = std::vector<double>(joint_velocities.at(i).data(), joint_velocities.at(i).data() + joint_velocities.at(i).rows());

                feedback->traj.points.push_back(point);

                i++;
            }
            feedback->cost = cost;
            goal_handle->publish_feedback(feedback);
        });

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        result->over = true;
        goal_handle->succeed(result);
    }
};
RCLCPP_COMPONENTS_REGISTER_NODE(PlannerActionServer)
