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

#include <mutex>
#include <memory>
#include <atomic>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/stop_trajectory_builder.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <pilz_control/cartesian_speed_monitor.h>
#include <pilz_control/traj_mode_manager.h>

namespace pilz_joint_trajectory_controller
{

template<class Segment> using TrajectoryPerJoint  = std::vector<Segment>;

template<class Segment>
static bool isStopMotionFinished(const std::vector<TrajectoryPerJoint<Segment>>& traj,
                                 const ros::Time& curr_uptime);

/**
 * @class PilzJointTrajectoryController
 * @brief Specialized controller that can be triggered by a service to
 * move the robot to its hold position and refuse further trajectories.
 */
template <class SegmentImpl, class HardwareInterface>
class PilzJointTrajectoryController
    : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
{
  public:
    typedef joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
                                                                                              JointTrajectoryController;
    typedef trajectory_msgs::JointTrajectory::ConstPtr                                          JointTrajectoryConstPtr;
    typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
    typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
    typedef joint_trajectory_controller::JointTrajectorySegment<SegmentImpl> Segment;
    typedef std::vector<Segment> TrajectoryPerJoint;
    typedef std::vector<TrajectoryPerJoint> Trajectory;
    typedef std::shared_ptr<Trajectory> TrajectoryPtr;

    PilzJointTrajectoryController();

    bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

    void starting(const ros::Time& time) override;

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


    bool handleMonitorCartesianSpeedRequest(std_srvs::SetBool::Request& req,
                                            std_srvs::SetBool::Response& res);

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

private:
    void updateFuncExtensionPoint(const typename JointTrajectoryController::Trajectory& curr_traj,
                                  const typename JointTrajectoryController::TimeData& time_data) override;
    bool isPlannedCartesianVelocityOK(const ros::Duration& period) const;
    void stopMotion(const ros::Time& curr_uptime);

    /**
     * @brief This function basically does what it's base class counterpart
     * JointTrajectoryController::preemptActiveGoal() does.
     * However, in contrast to JointTrajectoryController::preemptActiveGoal(), this function
     * only triggers the cancelling of the active goal. The actual execution of the
     * cancelling is left to a separate thread.
     *
     * @note In contrast to JointTrajectoryController::preemptActiveGoal() this function
     * is real-time safe.
     */
    void triggerCancellingOfActiveGoal();

private:
  private:
    ros::ServiceServer hold_position_service;
    ros::ServiceServer unhold_position_service;
    ros::ServiceServer is_executing_service_;
    ros::ServiceServer speed_limit_service_;

    std_srvs::TriggerRequest last_request_;

    std::unique_ptr<TrajProcessingModeManager> mode_ {
      std::unique_ptr<TrajProcessingModeManager>(new TrajProcessingModeManager())};

    std::unique_ptr<pilz_control::CartesianSpeedMonitor> cartesian_speed_monitor_;
    std::unique_ptr<joint_trajectory_controller::StopTrajectoryBuilder<SegmentImpl> > stop_traj_builder_;
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
    std::atomic<double> cartesian_speed_limit_ {0.0};

    //! A robot model is needed for the cartesian speed monitor. The loader must not be destroyed before it.
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
};

}  // namespace pilz_joint_trajectory_controller

#include <pilz_control/pilz_joint_trajectory_controller_impl.h>

#endif  //  PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_H
