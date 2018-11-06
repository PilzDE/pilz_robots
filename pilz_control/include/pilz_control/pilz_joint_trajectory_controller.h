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

#include <std_srvs/Trigger.h>

#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace pilz_joint_trajectory_controller
{

/**
 * @class PilzJointTrajectoryController Specialized controller that can be triggered by a service to 
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

    PilzJointTrajectoryController();

    bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    /**
     * @brief Service callback to force the controller into the hold position.
     *
     * @param response True on success.
     *
     * @return False if something went wrong. True otherwise.
     */
    bool handleHoldRequest(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

    /**
     * @brief Service callback to deactivate holding mode.
     *
     * @param response True on success.
     *
     * @return False if something went wrong. True otherwise.
     */
    bool handleUnHoldRequest(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

  protected:
    /**
     * @brief Called if new trajectory should be handled
     * Behaviour: Uses currently active strategy, either
     * updateStrategyDefault(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr)
     * or updateStrategyWhileHolding(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr)
     */
    bool updateTrajectoryCommand(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr) override;

    /**
     * @brief Hands received JointTrajectory to parent class, normal handling of trajectory
     */
    bool updateStrategyDefault(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr);

    /**
     * @brief Strategy applied if no new trajectory should be handled
     */
    bool updateStrategyWhileHolding(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr);

    void triggerMovementToHoldPosition();

  private:
    std::function<bool(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr)> active_update_strategy_;

    ros::ServiceServer hold_position_service;
    ros::ServiceServer unhold_position_service;

    std_srvs::TriggerRequest last_request_;

    /**
     * @brief Synchronizes hold/unhold and update trajectory function to avoid
     * threading problems.
     */
    std::mutex sync_mutex_;
};

}  // namespace pilz_joint_trajectory_controller

#include <pilz_control/pilz_joint_trajectory_controller_impl.h>

#endif  //  PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_H
