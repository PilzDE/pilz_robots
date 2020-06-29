/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_H
#define PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_H

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <boost/optional/optional_io.hpp>

#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/stop_trajectory_builder.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <pilz_control/cartesian_speed_monitor.h>
#include <pilz_control/traj_mode_manager.h>

namespace pilz_joint_trajectory_controller
{
template <class Segment>
using TrajectoryPerJoint = std::vector<Segment>;

/**
 * @brief Check if a trajectory is executed currently.
 *
 * @param traj Targeted trajectory.
 * @param curr_uptime Current uptime of the controller.
 * @note Times that preceed the trajectory start time are ignored here, so isTrajectoryExecuted() returns false
 * even if there is a current trajectory that will be executed in the future.
 * @return True if trajectory is executed currently, otherwise false.
 */
template <class Segment>
static bool isTrajectoryExecuted(const std::vector<TrajectoryPerJoint<Segment>>& traj, const ros::Time& curr_uptime);

/**
 * @class PilzJointTrajectoryController
 * @brief Specialized controller implementing ISO-10218-1 required features.
 *
 * Through the hold service the controller can be triggered to
 * move the robot to its hold position and refuse further trajectories.
 * The different modes of the controller (stopping, hold, unhold) are managed
 * by the TrajProcessingModeManager. In addition cartesian speed monitoring is realized
 * with the pilz_control::CartesianSpeedMonitor.
 */
template <class SegmentImpl, class HardwareInterface>
class PilzJointTrajectoryController
  : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
{
public:
  typedef joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
      JointTrajectoryController;
  typedef trajectory_msgs::JointTrajectory::ConstPtr JointTrajectoryConstPtr;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle> RealtimeGoalHandlePtr;
  typedef joint_trajectory_controller::JointTrajectorySegment<SegmentImpl> Segment;
  typedef std::vector<Segment> TrajectoryPerJoint;
  typedef std::vector<TrajectoryPerJoint> Trajectory;
  typedef std::shared_ptr<Trajectory> TrajectoryPtr;

  PilzJointTrajectoryController();

  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  /**
   * @brief Returns true if the controller currently is executing a trajectory. False otherwise.
   */
  bool is_executing();

  /**
   * @brief Service callback to force the controller into the hold position.
   *
   * @param request Dummy for triggering the service
   * @param response True on success.
   *
   * @return False if something went wrong. True otherwise.
   */
  bool handleHoldRequest(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

  /**
   * @brief Service callback to deactivate holding mode.
   *
   * @param request Dummy for triggering the service
   * @param response True on success.
   *
   * @return False if something went wrong. True otherwise.
   */
  bool handleUnHoldRequest(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

  /**
   * @brief Service callback for querying the controller activity.
   *
   * @param request Dummy for triggering the service
   * @param response success: True if the controller is currently executing a trajectory. False otherwise.
   *
   * @return False if something went wrong. True otherwise.
   */
  bool handleIsExecutingRequest(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

  /**
   * @brief Service callback for (de-)activate the cartesian speed monitoring.
   *
   * @param request If request.data==true the speed monitoring is activated, else it is deactivated.
   * @param response success: Always true.
   *
   * @return Always true.
   */
  bool handleMonitorCartesianSpeedRequest(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  /**
   * @brief Helper function to get paramter names to read
   *
   * @param param_name std::string This is where the param name will be written to.
   * @param joint_name std::sting Joint name as part of the param name.
   * @param suffix std::sting Suffix to be appended to the param name.
   */
  static void makeParamNameWithSuffix(std::string& param_name, const std::string& joint_name,
                                      const std::string& suffix);

  /**
   * @brief Get the Joint Acceleration Limits for each joint from the parameter server.
   *
   * If has_acceleration_limits is set to false acceleration limits will be set to 0.
   *
   * Function assumes parameter server naming prefix '/joint_limits/' for Joint Names.
   *
   * @param nh NodeHandle to access parameter server.
   * @param joint_names Vector of Strings for all joint names to get the acceleration limits for.
   * @return std::vector<boost::optional<double>> Requested acceleration limits as vector. Has the same length as param
   * 'joint_names'.
   *
   * @throw InvalidParameterException Requested values not found on param server.
   */
  static std::vector<boost::optional<double>> getJointAccelerationLimits(const ros::NodeHandle& nh,
                                                                         const std::vector<std::string>& joint_names);

protected:
  /**
   * @brief Called if new trajectory should be handled
   * Behaviour: Uses currently active strategy, either
   * updateStrategyDefault(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr)
   * or updateStrategyWhileHolding(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr)
   */
  bool updateTrajectoryCommand(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr,
                               std::string* error_string = 0) override;

  /**
   * @brief Hands received JointTrajectory to parent class, normal handling of trajectory
   */
  bool updateStrategyDefault(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr, std::string* error_string = 0);

  /**
   * @brief Strategy applied if no new trajectory should be handled
   */
  bool updateStrategyWhileHolding(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr, std::string* error_string = 0);

  void triggerMovementToHoldPosition();

  void trajectoryCommandCB(const JointTrajectoryConstPtr& msg) override;

private:
  /**
   * @brief Invoke cartesian speed monitoring and perform controlled stop in case of speed limit violation.
   *
   * The actual procedure depends on the current mode.
   * - unhold: check the (desired) velocity and trigger controller stop in case of speed limit violation.
   * - stopping: check if the stop trajectory execution is complete. If yes, trigger hold.
   * - hold: nothing to do.
   *
   * @param curr_traj Currently executed trajectory. Needed in order to detect a completed stop motion.
   * @param time_data Updated time data of the controller.
   */
  void updateFuncExtensionPoint(const typename JointTrajectoryController::Trajectory& curr_traj,
                                const typename JointTrajectoryController::TimeData& time_data) override;

  /**
   * @brief Check if planned update fullfilles all requirements on trajectory execution.
   *
   * @param period The time passed since the last update.
   *
   * @returns True if update can be performed, otherwise false.
   */
  bool isPlannedUpdateOK(const ros::Duration& period) const;

  /**
   * @brief Check acceleration limit. Ensure that trajectories are smooth enough.
   *
   * @param period The time passed since the last update.
   *
   * @returns False if one or more joints violate the acceleration limit, otherwise true.
   */
  bool isPlannedJointAccelerationOK(const ros::Duration& period) const;

  /**
   * @brief Trigger cartesian speed monitoring using the current and the desired joint states.
   *
   * @param period The time passed since the last update.
   *
   * @returns False if one or more links violate the Cartesian speed limit, otherwise true.
   */
  bool isPlannedCartesianVelocityOK(const ros::Duration& period) const;

  /**
   * @brief Cancel the currently active goal and trigger a controller stop.
   *
   * @param curr_uptime Current uptime of controller.
   */
  void stopMotion(const ros::Time& curr_uptime);

  /**
   * TODO: We should rather only trigger the cancelling of the active goal. The actual execution should be moved to a
   * separate thread. See realtime_tools::RealtimeServerGoalHandle and
   * https://github.com/ros-controls/ros_controllers/issues/174.
   */
  void cancelActiveGoal();

  /**
   * TODO: We should rather only trigger the aborting of the active goal. The actual execution should be moved to a
   * separate thread. See realtime_tools::RealtimeServerGoalHandle and
   * https://github.com/ros-controls/ros_controllers/issues/174.
   */
  void abortActiveGoal();

private:
  ros::ServiceServer hold_position_service;
  ros::ServiceServer unhold_position_service;
  ros::ServiceServer is_executing_service_;
  ros::ServiceServer monitor_cartesian_speed_service_;

  //! @brief Manages the different modes of the controller (stopping, hold, unhold).
  std::unique_ptr<TrajProcessingModeManager> mode_{ std::unique_ptr<TrajProcessingModeManager>(
      new TrajProcessingModeManager()) };

  std::unique_ptr<pilz_control::CartesianSpeedMonitor> cartesian_speed_monitor_;
  std::unique_ptr<joint_trajectory_controller::StopTrajectoryBuilder<SegmentImpl>> stop_traj_builder_;
  /**
   * @brief Stores the stop trajectory which is used in case the newly calculated desired value violates the
   * Cartesian path velocity restraint.
   */
  TrajectoryPtr stop_traj_velocity_violation_;

  /**
   * @brief Synchronizes hold/unhold and update trajectory function to avoid
   * threading problems.
   */
  std::mutex sync_mutex_;

  //! The currently max allowed speed for each frame on the Cartesian trajectory.
  std::atomic<double> cartesian_speed_limit_{ 0.0 };

  //! The max allowed acceleration for each joint.
  std::vector<boost::optional<double>> acceleration_joint_limits_;

  /**
   * @brief Used for loading a RobotModel for the CartesianSpeedMonitor.
   *
   * @note The RobotModelLoader uses a pluginlib::ClassLoader, which must not go out of scope
   * while the RobotModel is used, see http://wiki.ros.org/pluginlib. Since the RobotModel is injected into
   * the member cartesian_speed_monitor_, the loader has to be stored as well.
   */
  robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
};

}  // namespace pilz_joint_trajectory_controller

#include <pilz_control/pilz_joint_trajectory_controller_impl.h>

#endif  //  PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_H
