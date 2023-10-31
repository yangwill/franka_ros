// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include "std_msgs/Float64MultiArray.h"

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>

namespace franka_example_controllers {

class TorquePassthroughController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::EffortJointInterface,
                                           franka_hw::FrankaStateInterface,
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  Eigen::Matrix<double, 7, 1> clampTorques(
      Eigen::Matrix<double, 7, 1>& u_des) const;
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      Eigen::Matrix<double, 7, 1>& u_des,
      const Eigen::Matrix<double, 7, 1>& u_current) const;
  void handleTorqueCommand(const std_msgs::Float64MultiArray& msg);

  std::mutex u_des_mutex_;
  // hardware_interface::PositionJointInterface* position_joint_interface_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  Eigen::Matrix<double, 8, 1> u_des_;

  const double delta_u_max_{0.5};
  std::array<double, 7> initial_pose_{};
  ros::Subscriber torque_command_subscriber_;   // read torque commands sent from drake
  ros::Publisher joint_states_publisher_;
};

}  // namespace franka_example_controllers
