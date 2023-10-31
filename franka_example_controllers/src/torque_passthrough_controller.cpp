// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/torque_passthrough_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "sensor_msgs/JointState.h"
#include <ros/ros.h>

namespace franka_example_controllers {

bool TorquePassthroughController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  torque_command_subscriber_ = node_handle.subscribe(
      "/franka/franka_input", 1, &TorquePassthroughController::handleTorqueCommand, this,
      ros::TransportHints().reliable().tcpNoDelay());

  joint_states_publisher_ = node_handle.advertise<sensor_msgs::JointState>("/franka/joint_states", 1);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("TorquePassthroughController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "TorquePassthroughController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "TorquePassthroughController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "TorquePassthroughController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  /// Initialize torque interface
  auto* effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "TorquePassthroughController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "TorquePassthroughController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  /// Check Robot Start Position
  auto position_joint_interface = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface == nullptr) {
    ROS_ERROR(
        "TorquePassthroughController: Error getting position joint interface from hardware!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "TorquePassthroughController: Exception getting joint handles: " << e.what());
      return false;
    }
  }
  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "TorquePassthroughController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  u_des_.setZero();

  return true;
}

void TorquePassthroughController::starting(const ros::Time& /* time */) {
  std::lock_guard<std::mutex> u_des_mutex_lock(u_des_mutex_);
  u_des_.setZero();
}

void TorquePassthroughController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau(robot_state.tau_J.data());

  Eigen::Matrix<double, 7, 1> tau_d;
  tau_d.setZero();

  // Store previous desired values
  std::unique_lock<std::mutex> u_des_mutex_lock(u_des_mutex_);
  for (int i = 0; i < 7; i++){
    tau_d(i) = u_des_(i);
  }
  u_des_mutex_lock.unlock();

  // Get current torque values
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  clampTorques(tau_d);
  saturateTorqueRate(tau_d, tau_J_d);

  // send the torques to the hardware
  // echo the command to ensure the values are as intended
  for (size_t i = 0; i < 7; ++i) {
    // joint_handles_[i].setCommand(u_des_(i));
  }

  sensor_msgs::JointState msg;
  msg.position.resize(7);
  msg.velocity.resize(7);
  msg.effort.resize(7);
  for (size_t i = 0; i < 7; i++){
    msg.position[i] = q(i);
    msg.velocity[i] = dq(i);
    msg.effort[i] = tau(i);
  }
  joint_states_publisher_.publish(msg);
}

Eigen::Matrix<double, 7, 1> TorquePassthroughController::clampTorques(
    Eigen::Matrix<double, 7, 1>& u_des) const {
  Eigen::Matrix<double, 7, 1> tau_d_clamped{};

  double limit = 20;
  double wrist_limit = 2;

  for (size_t i = 0; i < 4; i++) {
    tau_d_clamped(i) = u_des(i);
    if (abs(u_des(i)) > limit) {
      double sign = u_des(i) / abs(u_des(i));
      tau_d_clamped(i) = sign * limit;
    }
  }
  for (size_t i = 4; i < 7; i++) {
    tau_d_clamped(i) = u_des(i);
    if (abs(u_des(i)) > wrist_limit) {
      double sign = u_des(i) / abs(u_des(i));
      tau_d_clamped(i) = sign * wrist_limit;
    }
  }
  return tau_d_clamped;
}

Eigen::Matrix<double, 7, 1> TorquePassthroughController::saturateTorqueRate(
    Eigen::Matrix<double, 7, 1>& u_des,
    const Eigen::Matrix<double, 7, 1>& u_current) const {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = u_des[i] - u_current[i];
    tau_d_saturated[i] =
        u_current[i] + std::max(std::min(difference, delta_u_max_), -delta_u_max_);
  }
  return tau_d_saturated;
}

void TorquePassthroughController::handleTorqueCommand(
    const std_msgs::Float64MultiArray& msg) {
  
  std::lock_guard<std::mutex> u_des_mutex_lock(u_des_mutex_);
  std::cout << "received torque message" << std::endl;
  for (int i = 0; i < 7; i++){
    u_des_(i) = msg.data[i];
    std::cout << "joint " << i << " torque: " << u_des_(i) << std::endl;
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::TorquePassthroughController,
                       controller_interface::ControllerBase)
