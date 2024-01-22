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

#include <Eigen/Core>

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <ergodic_sketching_msgs/PlannerAction.h>
#include "ergodic_sketching_ros/Planner.hpp"

class PlannerManager {
public:
    PlannerManager(const std::string& entry_point, const ros::NodeHandle node_handle, const Planner::Configuration& config, const std::vector<std::string>& joint_names) : joint_names_(joint_names) {
        planner_ = std::make_unique<Planner>(config);
        action_server_ = std::make_unique<actionlib::SimpleActionServer<ergodic_sketching_msgs::PlannerAction>>(
            node_handle, entry_point, [&](const ergodic_sketching_msgs::PlannerGoalConstPtr& goal) { this->execute(goal); }, false);
        action_server_->start();
    }

    void init() {}

    void execute(const ergodic_sketching_msgs::PlannerGoalConstPtr& goal) {
        ergodic_sketching_msgs::PlannerResult result;

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
            ergodic_sketching_msgs::PlannerFeedback feedback;
            feedback.traj.joint_names = joint_names_;
            feedback.traj.header.stamp = ros::Time::now();

            int i = 0;

            for (auto const& joint_position : joint_positions) {
                trajectory_msgs::JointTrajectoryPoint point;
                point.positions = std::vector<double>(joint_position.data(), joint_position.data() + joint_position.rows());
                point.velocities = std::vector<double>(joint_velocities.at(i).data(), joint_velocities.at(i).data() + joint_velocities.at(i).rows());

                feedback.traj.points.push_back(point);

                i++;
            }
            feedback.cost = cost;
            action_server_->publishFeedback(feedback);
        });

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        result.over = true;
        action_server_->setSucceeded(result);
    }

private:
    std::unique_ptr<Planner> planner_;
    std::unique_ptr<actionlib::SimpleActionServer<ergodic_sketching_msgs::PlannerAction>> action_server_;
    std::vector<std::string> joint_names_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node_handle("~");

    Planner::Configuration config;

    if (!node_handle.getParam("/robot_description", config.urdf)) {
        ROS_ERROR("No parameters /robot_description");
        return 1;
    }

    if (!node_handle.getParam("base_frame", config.base_frame)) {
        ROS_ERROR("No parameters %s/base_frame", node_handle.getNamespace().c_str());
        return 1;
    }

    if (!node_handle.getParam("tip_frame", config.tip_frame)) {
        ROS_ERROR("No parameters %s/tip_frame", node_handle.getNamespace().c_str());
        return 1;
    }

    if (!node_handle.getParam("order", config.order)) {
        ROS_ERROR("No parameters %s/order", node_handle.getNamespace().c_str());
        return 1;
    }

    if (!node_handle.getParam("dof", config.dof)) {
        ROS_ERROR("No parameters %s/dof", node_handle.getNamespace().c_str());
        return 1;
    }

    if (!node_handle.getParam("rfactor", config.rfactor)) {
        ROS_ERROR("No parameters %s/rfactor", node_handle.getNamespace().c_str());
        return 1;
    }

    if (!node_handle.getParam("init_phase_length", config.init_phase_length)) {
        ROS_ERROR("No parameters %s/init_phase_length", node_handle.getNamespace().c_str());
        return 1;
    }

    config.nb_ctrl_var = config.dof;
    config.nb_state_var = 7;
    config.nb_iter = 100;
    config.dt = 0.01;
    config.batch_size = 7500;

    std::vector<double> q_max, q_min, dq_max, dq_min, q0, orn_thresh;

    if (!node_handle.getParam("q_max", q_max)) {
        ROS_ERROR("No parameters %s/q_max", node_handle.getNamespace().c_str());
        return 1;
    }
    config.q_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_max.data(), q_max.size());

    if (!node_handle.getParam("q_min", q_min)) {
        ROS_ERROR("No parameters %s/q_min", node_handle.getNamespace().c_str());
        return 1;
    }
    config.q_min = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_min.data(), q_min.size());

    if (!node_handle.getParam("dq_max", dq_max)) {
        ROS_ERROR("No parameters %s/dq_max", node_handle.getNamespace().c_str());
        return 1;
    }
    config.dq_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(dq_max.data(), dq_max.size());

    if (!node_handle.getParam("dq_min", dq_min)) {
        ROS_ERROR("No parameters %s/dq_min", node_handle.getNamespace().c_str());
        return 1;
    }
    config.dq_min = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(dq_min.data(), dq_min.size());

    if (!node_handle.getParam("q0", q0)) {
        ROS_ERROR("No parameters %s/q0", node_handle.getNamespace().c_str());
        return 1;
    }
    config.q0 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q0.data(), q0.size());

    if (!node_handle.getParam("orn_thresh", orn_thresh)) {
        ROS_ERROR("No parameters %s/orn_thresh", node_handle.getNamespace().c_str());
        return 1;
    }
    config.orn_thresh = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(orn_thresh.data(), orn_thresh.size());

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names)) {
        ROS_ERROR("No parameters %s/joint_names", node_handle.getNamespace().c_str());
        return 1;
    }

    config.dq0 = Eigen::VectorXd::Zero(config.dof);

    PlannerManager manager("plan", node_handle, config, joint_names);
    manager.init();

    ros::spin();
}
