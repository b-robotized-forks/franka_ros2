// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cassert>
#include <mutex>

#include <franka/control_tools.h>
#include <franka/rate_limiting.h>
#include <research_interface/robot/rbk_types.h>
#include <rclcpp/logging.hpp>

#include "franka_hardware/robot.hpp"


// --- HACK BEGIN ---
namespace {
const std::string LEFT_ROBOT_URDF = R"RAW_URDF(
<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document is adapted for the left_robot instance (Dynamics Only)         | -->
<!-- =================================================================================== -->
<robot name="left_fp3">
  <link name="left_base"/>
  
  <joint name="left_base_joint" type="fixed">
    <parent link="left_base"/>
    <child link="left_link0"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>

  <link name="left_link0"/>

  <link name="left_link1">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0000004128 -0.0181251324 -0.0386035970"/>
      <mass value="2.9274653454"/>
      <inertia ixx="0.0186029651" ixy="1.3296e-05" ixz="-0.0001140944" iyy="0.0181195421" iyz="5.33017e-05" izz="0.0053883661"/>
    </inertial>
  </link>

  <joint name="left_joint1" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 0.333"/>
    <parent link="left_link0"/>
    <child link="left_link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-2.7437" upper="2.7437" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.7437" soft_upper_limit="2.7437"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="left_link2">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0031828864 -0.0743221644 0.0088146084"/>
      <mass value="2.9355370338"/>
      <inertia ixx="0.0248426123" ixy="0.0012852153" ixz="0.0047668011" iyy="0.0200510561" iyz="-0.0077993576" izz="0.0448935091"/>
    </inertial>
  </link>

  <joint name="left_joint2" type="revolute">
    <origin rpy="-1.570796326794897 0 0" xyz="0 0 0"/>
    <parent link="left_link1"/>
    <child link="left_link2"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-1.7837" upper="1.7837" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-1.7837" soft_upper_limit="1.7837"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="left_link3">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0407015686 -0.0048200565 -0.0289730823"/>
      <mass value="2.2449013699"/>
      <inertia ixx="0.0206960046" ixy="0.0006503378" ixz="-0.0086867454" iyy="0.0129617222" iyz="-0.0046382675" izz="0.0083748603"/>
    </inertial>
  </link>

  <joint name="left_joint3" type="revolute">
    <origin rpy="1.570796326794897 0 0" xyz="0 -0.316 0"/>
    <parent link="left_link2"/>
    <child link="left_link3"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-2.9007" upper="2.9007" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.9007" soft_upper_limit="2.9007"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="left_link4">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0459100965 0.0630492960 -0.0085187868"/>
      <mass value="2.6155955791"/>
      <inertia ixx="0.0189656777" ixy="0.0087442604" ixz="0.0154882253" iyy="0.021260554" iyz="-0.0050592943" izz="0.0216050853"/>
    </inertial>
  </link>

  <joint name="left_joint4" type="revolute">
    <origin rpy="1.570796326794897 0 0" xyz="0.0825 0 0"/>
    <parent link="left_link3"/>
    <child link="left_link4"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-3.0421" upper="-0.1518" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-3.0421" soft_upper_limit="-0.1518"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="left_link5">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0016039605 0.0292536262 -0.0972965990"/>
      <mass value="2.3271207594"/>
      <inertia ixx="0.0274316356" ixy="-0.0054626383" ixz="-0.0045776221" iyy="0.0250087792" iyz="0.0072115131" izz="0.0024232492"/>
    </inertial>
  </link>

  <joint name="left_joint5" type="revolute">
    <origin rpy="-1.570796326794897 0 0" xyz="-0.0825 0.384 0"/>
    <parent link="left_link4"/>
    <child link="left_link5"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-2.8065" upper="2.8065" velocity="5.26"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8065" soft_upper_limit="2.8065"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="left_link6">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0597131221 -0.0410294666 -0.0101692726"/>
      <mass value="1.8170376524"/>
      <inertia ixx="0.000724256" ixy="0.0021884556" ixz="0.0004615138" iyy="0.0072530249" iyz="-0.0006002451" izz="0.0065288729"/>
    </inertial>
  </link>

  <joint name="left_joint6" type="revolute">
    <origin rpy="1.570796326794897 0 0" xyz="0 0 0"/>
    <parent link="left_link5"/>
    <child link="left_link6"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="0.5445" upper="4.5169" velocity="4.18"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="0.5445" soft_upper_limit="4.5169"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="left_link7">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0045225817 0.0086261921 -0.0161633251"/>
      <mass value="0.6271432862"/>                                                                                                                  
      <inertia ixx="4.039e-07" ixy="1.875e-07" ixz="-1.593e-07" iyy="2.351e-07" iyz="-2.084e-07" izz="2.838e-07"/>
    </inertial>
  </link>

  <joint name="left_joint7" type="revolute">
    <origin rpy="1.570796326794897 0 0" xyz="0.088 0 0"/>
    <parent link="left_link6"/>
    <child link="left_link7"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-3.0159" upper="3.0159" velocity="5.26"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-3.0159" soft_upper_limit="3.0159"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="left_link8"/>
  
  <joint name="left_joint8" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.107"/>
    <parent link="left_link7"/>
    <child link="left_link8"/>
  </joint>
</robot>
)RAW_URDF";

const std::string RIGHT_ROBOT_URDF = R"RAW_URDF(
<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document is adapted for the right_robot instance (Dynamics Only)        | -->
<!-- =================================================================================== -->
<robot name="right_fp3">
  <link name="right_base"/>
  
  <joint name="right_base_joint" type="fixed">
    <parent link="right_base"/>
    <child link="right_link0"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>

  <link name="right_link0"/>

  <link name="right_link1">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0000004128 -0.0181251324 -0.0386035970"/>
      <mass value="2.9274653454"/>
      <inertia ixx="0.0186029651" ixy="1.3296e-05" ixz="-0.0001140944" iyy="0.0181195421" iyz="5.33017e-05" izz="0.0053883661"/>
    </inertial>
  </link>

  <joint name="right_joint1" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 0.333"/>
    <parent link="right_link0"/>
    <child link="right_link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-2.7437" upper="2.7437" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.7437" soft_upper_limit="2.7437"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="right_link2">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0031828864 -0.0743221644 0.0088146084"/>
      <mass value="2.9355370338"/>
      <inertia ixx="0.0248426123" ixy="0.0012852153" ixz="0.0047668011" iyy="0.0200510561" iyz="-0.0077993576" izz="0.0448935091"/>
    </inertial>
  </link>

  <joint name="right_joint2" type="revolute">
    <origin rpy="-1.570796326794897 0 0" xyz="0 0 0"/>
    <parent link="right_link1"/>
    <child link="right_link2"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-1.7837" upper="1.7837" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-1.7837" soft_upper_limit="1.7837"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="right_link3">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0407015686 -0.0048200565 -0.0289730823"/>
      <mass value="2.2449013699"/>
      <inertia ixx="0.0206960046" ixy="0.0006503378" ixz="-0.0086867454" iyy="0.0129617222" iyz="-0.0046382675" izz="0.0083748603"/>
    </inertial>
  </link>

  <joint name="right_joint3" type="revolute">
    <origin rpy="1.570796326794897 0 0" xyz="0 -0.316 0"/>
    <parent link="right_link2"/>
    <child link="right_link3"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-2.9007" upper="2.9007" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.9007" soft_upper_limit="2.9007"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="right_link4">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0459100965 0.0630492960 -0.0085187868"/>
      <mass value="2.6155955791"/>
      <inertia ixx="0.0189656777" ixy="0.0087442604" ixz="0.0154882253" iyy="0.021260554" iyz="-0.0050592943" izz="0.0216050853"/>
    </inertial>
  </link>

  <joint name="right_joint4" type="revolute">
    <origin rpy="1.570796326794897 0 0" xyz="0.0825 0 0"/>
    <parent link="right_link3"/>
    <child link="right_link4"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-3.0421" upper="-0.1518" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-3.0421" soft_upper_limit="-0.1518"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="right_link5">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0016039605 0.0292536262 -0.0972965990"/>
      <mass value="2.3271207594"/>
      <inertia ixx="0.0274316356" ixy="-0.0054626383" ixz="-0.0045776221" iyy="0.0250087792" iyz="0.0072115131" izz="0.0024232492"/>
    </inertial>
  </link>

  <joint name="right_joint5" type="revolute">
    <origin rpy="-1.570796326794897 0 0" xyz="-0.0825 0.384 0"/>
    <parent link="right_link4"/>
    <child link="right_link5"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-2.8065" upper="2.8065" velocity="5.26"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8065" soft_upper_limit="2.8065"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="right_link6">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0597131221 -0.0410294666 -0.0101692726"/>
      <mass value="1.8170376524"/>
      <inertia ixx="0.000724256" ixy="0.0021884556" ixz="0.0004615138" iyy="0.0072530249" iyz="-0.0006002451" izz="0.0065288729"/>
    </inertial>
  </link>

  <joint name="right_joint6" type="revolute">
    <origin rpy="1.570796326794897 0 0" xyz="0 0 0"/>
    <parent link="right_link5"/>
    <child link="right_link6"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="0.5445" upper="4.5169" velocity="4.18"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="0.5445" soft_upper_limit="4.5169"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="right_link7">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0045225817 0.0086261921 -0.0161633251"/>
      <mass value="0.6271432862"/>                                                                                                                  
      <inertia ixx="4.039e-07" ixy="1.875e-07" ixz="-1.593e-07" iyy="2.351e-07" iyz="-2.084e-07" izz="2.838e-07"/>
    </inertial>
  </link>

  <joint name="right_joint7" type="revolute">
    <origin rpy="1.570796326794897 0 0" xyz="0.088 0 0"/>
    <parent link="right_link6"/>
    <child link="right_link7"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-3.0159" upper="3.0159" velocity="5.26"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-3.0159" soft_upper_limit="3.0159"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.0" mu_coulomb="0" mu_viscous="16"/>
  </joint>

  <link name="right_link8"/>
  
  <joint name="right_joint8" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.107"/>
    <parent link="right_link7"/>
    <child link="right_link8"/>
  </joint>
</robot>
)RAW_URDF";
}
// --- HACK END ---

namespace franka_hardware {

Robot::Robot(std::unique_ptr<franka::Robot> robot, std::unique_ptr<Model> model)
    : robot_(std::move(robot)), franka_hardware_model_(std::move(model)), logger_(rclcpp::get_logger("franka_hw_interface")) {}

Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) : logger_(logger)
{
  franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
  if (!franka::hasRealtimeKernel()) {
    rt_config = franka::RealtimeConfig::kIgnore;
    RCLCPP_WARN(
        logger,
        "You are not using a real-time kernel. Using a real-time kernel is strongly "
        "recommended! Information about how to set up a real-time kernel can be found here: "
        "https://frankaemika.github.io/docs/"
        "installation_linux.html#setting-up-the-real-time-kernel");
  }
  robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
  // --- HACK ---
  // depending on IP address, load hardcoded robot urdf
  std::unique_ptr<franka::Model> franka_model_ptr;
  
  if (robot_ip == "192.168.0.1") {
    franka_model_ptr = std::make_unique<franka::Model>(robot_->loadModel(LEFT_ROBOT_URDF));
  } 
  else if (robot_ip == "192.168.0.3") {
    franka_model_ptr = std::make_unique<franka::Model>(robot_->loadModel(RIGHT_ROBOT_URDF));
  } 
  else {
    // if no IP, fall back to previous functionality
    franka_model_ptr = std::make_unique<franka::Model>(robot_->loadModel());
  }
  franka_hardware_model_ = std::make_unique<Model>(franka_model_ptr.release());
  // --- HACK END ---

}

Robot::~Robot() {
  stopRobot();
}

franka::RobotState Robot::readOnce() {
  if (pause_read_requested_.load()) {
    std::unique_lock<std::mutex> sync_lock(read_sync_mutex_);
    
    // Signal we are pausing
    read_is_paused_ = true;
    read_sync_cv_.notify_all(); 
    
    // WAIT here until the guard is destroyed
    read_sync_cv_.wait(sync_lock, [this]{ return !pause_read_requested_.load(); });
    
    read_is_paused_ = false;
    // Return cached state to keep the loop valid
    return current_state_;
  }

  std::lock_guard<std::mutex> lock(control_mutex_);
  if (!active_control_) {
    current_state_ = robot_->readOnce();
  } else {
    current_state_ = readOnceActiveControl();
  }
  return current_state_;
}

void Robot::pauseBlockingRead() {
  RCLCPP_INFO(logger_, "[Guard] Requesting read pause...");
  pause_read_requested_.store(true);
  
  // Wait for the read loop to actually yield
  std::unique_lock<std::mutex> lock(read_sync_mutex_);
  read_sync_cv_.wait(lock, [this]{ return read_is_paused_; });
  RCLCPP_INFO(logger_, "[Guard] Read loop paused. Mutex free.");
}

void Robot::resumeBlockingRead() {
  RCLCPP_INFO(logger_, "[Guard] Resuming read loop...");
  {
    std::lock_guard<std::mutex> lock(read_sync_mutex_);
    pause_read_requested_.store(false);
  }
  read_sync_cv_.notify_all();
}

void Robot::stopRobot() {
 
  if (active_control_) {
    effort_interface_active_ = false;
    joint_velocity_interface_active_ = false;
    joint_position_interface_active_ = false;
    cartesian_velocity_interface_active_ = false;
    cartesian_pose_interface_active_ = false;
    active_control_.reset();
  }
  RCLCPP_INFO(logger_, "[DEBUG] stopRobot END");
}

void Robot::writeOnce(const std::array<double, 7>& joint_commands) {
  if (!active_control_) {
    throw std::runtime_error("Control hasn't been started");
  }
  if (effort_interface_active_) {
    writeOnceEfforts(joint_commands);
  } else if (joint_velocity_interface_active_) {
    writeOnceJointVelocities(joint_commands);
  } else if (joint_position_interface_active_) {
    writeOnceJointPositions(joint_commands);
  }
}

void Robot::writeOnceEfforts(const std::array<double, 7>& efforts) {
  std::lock_guard<std::mutex> lock(control_mutex_);

  auto torque_command = franka::Torques(efforts);
  if (torque_command_rate_limiter_active_) {
    torque_command.tau_J =
        franka::limitRate(franka::kMaxTorqueRate, torque_command.tau_J, current_state_.tau_J_d);
  }
  active_control_->writeOnce(torque_command);
}

void Robot::writeOnceJointVelocities(const std::array<double, 7>& velocities) {
  std::lock_guard<std::mutex> lock(control_mutex_);

  auto velocity_command = franka::JointVelocities(velocities);

  // If you are experiencing issues with robot error. You can try activating the rate limiter.
  // Rate limiter is default deactivated.
  if (velocity_command_rate_limit_active_) {
    velocity_command.dq = franka::limitRate(
        franka::computeUpperLimitsJointVelocity(current_state_.q_d),
        franka::computeLowerLimitsJointVelocity(current_state_.q_d), franka::kMaxJointAcceleration,
        franka::kMaxJointJerk, velocity_command.dq, current_state_.dq_d, current_state_.ddq_d);
  }

  active_control_->writeOnce(velocity_command);
}

void Robot::writeOnceJointPositions(const std::array<double, 7>& positions) {
  std::lock_guard<std::mutex> lock(control_mutex_);

  auto position_command = franka::JointPositions(positions);

  // If you are experiencing issues with robot error. You can try activating the rate limiter.
  // Rate limiter is default deactivated.
  if (joint_position_command_low_pass_filter_active_) {
    for (size_t i = 0; i < 7; i++) {
      position_command.q.at(i) =
          franka::lowpassFilter(franka::kDeltaT, position_command.q.at(i), current_state_.q_d.at(i),
                                low_pass_filter_cut_off_freq);
    }
  }
  if (joint_position_command_rate_limit_active_) {
    position_command.q =
        franka::limitRate(franka::computeUpperLimitsJointVelocity(current_state_.q_d),
                          franka::computeLowerLimitsJointVelocity(current_state_.q_d),
                          franka::kMaxJointAcceleration, franka::kMaxJointJerk, position_command.q,
                          current_state_.q_d, current_state_.dq_d, current_state_.ddq_d);
  }
  active_control_->writeOnce(position_command);
}

franka::CartesianVelocities Robot::preProcessCartesianVelocities(
    const franka::CartesianVelocities& velocity_command) {
  franka::CartesianVelocities filtered_velocity_command = velocity_command;
  if (cartesian_velocity_low_pass_filter_active_) {
    for (size_t i = 0; i < 6; i++) {
      filtered_velocity_command.O_dP_EE.at(i) =
          franka::lowpassFilter(franka::kDeltaT, velocity_command.O_dP_EE.at(i),
                                current_state_.O_dP_EE_c.at(i), low_pass_filter_cut_off_freq);
    }
  }

  // If you are experiencing issues with robot error. You can try activating the rate
  // limiter. Rate limiter is default deactivated (cartesian_velocity_command_rate_limit_active_)
  if (cartesian_velocity_command_rate_limit_active_) {
    filtered_velocity_command.O_dP_EE = franka::limitRate(
        franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
        franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
        franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk, velocity_command.O_dP_EE,
        current_state_.O_dP_EE_c, current_state_.O_ddP_EE_c);
  }

  return filtered_velocity_command;
}

franka::CartesianPose Robot::preProcessCartesianPose(const franka::CartesianPose& cartesian_pose) {
  franka::CartesianPose filtered_cartesian_pose = cartesian_pose;

  if (cartesian_pose_low_pass_filter_active_) {
    filtered_cartesian_pose.O_T_EE =
        franka::cartesianLowpassFilter(franka::kDeltaT, filtered_cartesian_pose.O_T_EE,
                                       current_state_.O_T_EE_c, low_pass_filter_cut_off_freq);
  }

  if (cartesian_pose_command_rate_limit_active_) {
    filtered_cartesian_pose.O_T_EE =
        franka::limitRate(franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                          franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                          franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk,
                          filtered_cartesian_pose.O_T_EE, current_state_.O_T_EE_c,
                          current_state_.O_dP_EE_c, current_state_.O_ddP_EE_c);
  }

  return filtered_cartesian_pose;
}

void Robot::writeOnce(const std::array<double, 6>& cartesian_velocities) {
  if (!active_control_) {
    throw std::runtime_error("Control hasn't been started");
  }

  std::lock_guard<std::mutex> lock(control_mutex_);

  auto velocity_command = franka::CartesianVelocities(cartesian_velocities);
  auto filtered_velocity_command = preProcessCartesianVelocities(velocity_command);
  active_control_->writeOnce(filtered_velocity_command);
}

void Robot::writeOnce(const std::array<double, 6>& cartesian_velocities,
                      const std::array<double, 2>& elbow_command) {
  if (!active_control_) {
    throw std::runtime_error("Control hasn't been started");
  }

  std::lock_guard<std::mutex> lock(control_mutex_);

  auto velocity_command = franka::CartesianVelocities(cartesian_velocities, elbow_command);
  auto filtered_velocity_command = preProcessCartesianVelocities(velocity_command);

  active_control_->writeOnce(filtered_velocity_command);
}

void Robot::writeOnce(const std::array<double, 16>& cartesian_pose) {
  if (!active_control_) {
    throw std::runtime_error("Control hasn't been started");
  }

  std::lock_guard<std::mutex> lock(control_mutex_);

  auto pose_command = franka::CartesianPose(cartesian_pose);
  auto filtered_pose = preProcessCartesianPose(pose_command);

  active_control_->writeOnce(filtered_pose);
}

void Robot::writeOnce(const std::array<double, 16>& cartesian_pose,
                      const std::array<double, 2>& elbow_command) {
  if (!active_control_) {
    throw std::runtime_error("Control hasn't been started");
  }

  std::lock_guard<std::mutex> lock(control_mutex_);

  auto pose_command = franka::CartesianPose(cartesian_pose, elbow_command);

  auto filtered_pose = preProcessCartesianPose(pose_command);

  active_control_->writeOnce(filtered_pose);
}

franka::RobotState Robot::readOnceActiveControl() {
  // When controller is active use active control to read the robot state
  const auto [current_state, _] = active_control_->readOnce();
  return current_state;
}

franka_hardware::Model* Robot::getModel() {
  return franka_hardware_model_.get();
}

void Robot::initializeTorqueInterface() {
  RCLCPP_INFO(logger_, "[DEBUG] initializeTorqueInterface START");
  try {
    active_control_ = robot_->startTorqueControl();
  } catch (const franka::InvalidOperationException& e) {
    RCLCPP_INFO(logger_, "initializeTorqueInterface: InvalidOperationException - Mutex contention detected!" );
    throw;
  } catch (const franka::ControlException& e) {
    robot_->automaticErrorRecovery();
    active_control_ = robot_->startTorqueControl();
  }
  effort_interface_active_ = true;
  RCLCPP_INFO(logger_, "[DEBUG] initializeTorqueInterface END");
}

void Robot::initializeJointVelocityInterface() {
  RCLCPP_INFO(logger_, "[DEBUG] initializeJointVelocityInterface START");
  try {
    active_control_ = robot_->startJointVelocityControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
  } catch (const franka::InvalidOperationException& e) {
    RCLCPP_INFO(logger_, "initializeJointVelocityInterface: InvalidOperationException - Mutex contention detected!" );
    throw;
  } catch (const franka::ControlException& e) {
    robot_->automaticErrorRecovery();
    active_control_ = robot_->startJointVelocityControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
  }

  joint_velocity_interface_active_ = true;
  RCLCPP_INFO(logger_, "[DEBUG] initializeJointVelocityInterface END");
}

void Robot::initializeJointPositionInterface() {
  RCLCPP_INFO(logger_, "[DEBUG] initializeJointPositionInterface START");
  try {
    active_control_ = robot_->startJointPositionControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
  } catch (const franka::InvalidOperationException& e) {
    RCLCPP_INFO(logger_, "initializeJointPositionInterface: InvalidOperationException - Mutex contention detected!" );
    throw;
  } catch (const franka::ControlException& e) {
    robot_->automaticErrorRecovery();
    active_control_ = robot_->startJointPositionControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
  }

  joint_position_interface_active_ = true;
  RCLCPP_INFO(logger_, "[DEBUG] initializeJointPositionInterface END");
}

void Robot::initializeCartesianVelocityInterface() {
  RCLCPP_INFO(logger_, "[DEBUG] initializeCartesianVelocityInterface START");
  try {
    active_control_ = robot_->startCartesianVelocityControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
  } catch (const franka::InvalidOperationException& e) {
    RCLCPP_INFO(logger_, "initializeCartesianVelocityInterface: InvalidOperationException - Mutex contention detected!" );
    throw;
  } catch (const franka::ControlException& e) {
    robot_->automaticErrorRecovery();
    active_control_ = robot_->startCartesianVelocityControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
  }
  cartesian_velocity_interface_active_ = true;
  RCLCPP_INFO(logger_, "[DEBUG] initializeCartesianVelocityInterface END");
}

void Robot::initializeCartesianPoseInterface() {
  RCLCPP_INFO(logger_, "[DEBUG] initializeCartesianPoseInterface START");
  try {
    active_control_ = robot_->startCartesianPoseControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
  } catch (const franka::InvalidOperationException& e) {
    RCLCPP_INFO(logger_, "initializeCartesianPoseInterface: InvalidOperationException - Mutex contention detected!" );
    throw;
  } catch (const franka::ControlException& e) {
    robot_->automaticErrorRecovery();
    active_control_ = robot_->startCartesianPoseControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
  }
  cartesian_pose_interface_active_ = true;
  RCLCPP_INFO(logger_, "[DEBUG] initializeCartesianPoseInterface END");
}

void Robot::setJointStiffness(const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& req) {
  RCLCPP_INFO(logger_, "[DEBUG] Service: setJointStiffness START");
  std::lock_guard<std::mutex> lock(write_mutex_);
  std::array<double, 7> joint_stiffness{};
  std::copy(req->joint_stiffness.cbegin(), req->joint_stiffness.cend(), joint_stiffness.begin());
  robot_->setJointImpedance(joint_stiffness);
  RCLCPP_INFO(logger_, "[DEBUG] Service: setJointStiffness END");
}

void Robot::setCartesianStiffness(
    const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& req) {
  RCLCPP_INFO(logger_, "[DEBUG] Service: setCartesianStiffness START");
  std::lock_guard<std::mutex> lock(write_mutex_);
  std::array<double, 6> cartesian_stiffness{};
  std::copy(req->cartesian_stiffness.cbegin(), req->cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  robot_->setCartesianImpedance(cartesian_stiffness);
  RCLCPP_INFO(logger_, "[DEBUG] Service: setCartesianStiffness END");
}

void Robot::setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& req) {
  RCLCPP_INFO(logger_, "[DEBUG] Service: setLoad START");
  std::lock_guard<std::mutex> lock(write_mutex_);
  double mass(req->mass);
  std::array<double, 3> center_of_mass{};  // NOLINT [readability-identifier-naming]
  std::copy(req->center_of_mass.cbegin(), req->center_of_mass.cend(), center_of_mass.begin());
  std::array<double, 9> load_inertia{};
  std::copy(req->load_inertia.cbegin(), req->load_inertia.cend(), load_inertia.begin());

  robot_->setLoad(mass, center_of_mass, load_inertia);
  RCLCPP_INFO(logger_, "[DEBUG] Service: setLoad END");
}

void Robot::setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& req) {
  RCLCPP_INFO(logger_, "[DEBUG] Service: setTCPFrame START");
  std::lock_guard<std::mutex> lock(write_mutex_);

  std::array<double, 16> transformation{};  // NOLINT [readability-identifier-naming]
  std::copy(req->transformation.cbegin(), req->transformation.cend(), transformation.begin());
  robot_->setEE(transformation);
  RCLCPP_INFO(logger_, "[DEBUG] Service: setTCPFrame END");
}

void Robot::setStiffnessFrame(const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& req) {
  RCLCPP_INFO(logger_, "[DEBUG] Service: setStiffnessFrame START");
  std::lock_guard<std::mutex> lock(write_mutex_);

  std::array<double, 16> transformation{};
  std::copy(req->transformation.cbegin(), req->transformation.cend(), transformation.begin());
  robot_->setK(transformation);
  RCLCPP_INFO(logger_, "[DEBUG] Service: setStiffnessFrame END");
}

void Robot::setForceTorqueCollisionBehavior(
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  RCLCPP_INFO(logger_, "[DEBUG] Service: setForceTorqueCollisionBehavior START");

  std::array<double, 7> lower_torque_thresholds_nominal{};
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal{};
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_nominal{};
  std::copy(req->lower_force_thresholds_nominal.cbegin(),
            req->lower_force_thresholds_nominal.cend(), lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal{};
  std::copy(req->upper_force_thresholds_nominal.cbegin(),
            req->upper_force_thresholds_nominal.cend(), upper_force_thresholds_nominal.begin());

  robot_->setCollisionBehavior(lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                               lower_force_thresholds_nominal, upper_force_thresholds_nominal);
  RCLCPP_INFO(logger_, "[DEBUG] Service: setForceTorqueCollisionBehavior END");
}

void Robot::setFullCollisionBehavior(
    const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& req) {
  RCLCPP_INFO(logger_, "[DEBUG] Service: setFullCollisionBehavior START");
  std::lock_guard<std::mutex> lock(write_mutex_);

  std::array<double, 7> lower_torque_thresholds_acceleration{};
  std::copy(req->lower_torque_thresholds_acceleration.cbegin(),
            req->lower_torque_thresholds_acceleration.cend(),
            lower_torque_thresholds_acceleration.begin());
  std::array<double, 7> upper_torque_thresholds_acceleration{};
  std::copy(req->upper_torque_thresholds_acceleration.cbegin(),
            req->upper_torque_thresholds_acceleration.cend(),
            upper_torque_thresholds_acceleration.begin());
  std::array<double, 7> lower_torque_thresholds_nominal{};
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal{};
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_acceleration{};
  std::copy(req->lower_force_thresholds_acceleration.cbegin(),
            req->lower_force_thresholds_acceleration.cend(),
            lower_force_thresholds_acceleration.begin());
  std::array<double, 6> upper_force_thresholds_acceleration{};
  std::copy(req->upper_force_thresholds_acceleration.cbegin(),
            req->upper_force_thresholds_acceleration.cend(),
            upper_force_thresholds_acceleration.begin());
  std::array<double, 6> lower_force_thresholds_nominal{};
  std::copy(req->lower_force_thresholds_nominal.cbegin(),
            req->lower_force_thresholds_nominal.cend(), lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal{};
  std::copy(req->upper_force_thresholds_nominal.cbegin(),
            req->upper_force_thresholds_nominal.cend(), upper_force_thresholds_nominal.begin());
  robot_->setCollisionBehavior(
      lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
      lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
      lower_force_thresholds_nominal, upper_force_thresholds_nominal);
  RCLCPP_INFO(logger_, "[DEBUG] Service: setFullCollisionBehavior END");
}

void Robot::automaticErrorRecovery() {
  RCLCPP_INFO(logger_, "[DEBUG] automaticErrorRecovery START");
  robot_->automaticErrorRecovery();
  RCLCPP_INFO(logger_, "[DEBUG] automaticErrorRecovery END");
}

}  // namespace franka_hardware
