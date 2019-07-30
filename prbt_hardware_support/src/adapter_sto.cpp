/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <prbt_hardware_support/adapter_sto.h>

#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/modbus_msg_sto_wrapper.h>
#include <prbt_hardware_support/wait_for_service.h>

namespace prbt_hardware_support
{

const std::string AdapterSto::HOLD_SERVICE = "manipulator_joint_trajectory_controller/hold";
const std::string AdapterSto::UNHOLD_SERVICE = "manipulator_joint_trajectory_controller/unhold";
const std::string AdapterSto::RECOVER_SERVICE = "driver/recover";
const std::string AdapterSto::HALT_SERVICE = "driver/halt";

AdapterSto::AdapterSto(ros::NodeHandle &nh)
{
  waitForService(HALT_SERVICE);
  waitForService(HOLD_SERVICE);

  if(!ros::service::waitForService(UNHOLD_SERVICE, ros::Duration(WAIT_FOR_SERVICE_TIMEOUT_S)))
  {
    ROS_WARN_STREAM("Controller service " << UNHOLD_SERVICE
                    << " not available. STO adapter node can not function properly. You can't unhold the controller"
                       " once it went to hold mode.");
  }
  if(!ros::service::waitForService(RECOVER_SERVICE, ros::Duration(WAIT_FOR_SERVICE_TIMEOUT_S)))
  {
    ROS_WARN_STREAM("Driver service " << RECOVER_SERVICE
                    << " not available. Driver won't auto-recover after STO clearance.");
  }


  // Attach to services (all should be available now)
  halt_srv_client_ = nh.serviceClient<std_srvs::Trigger>(HALT_SERVICE);
  hold_srv_client_ = nh.serviceClient<std_srvs::Trigger>(HOLD_SERVICE);
  unhold_srv_client_ = nh.serviceClient<std_srvs::Trigger>(UNHOLD_SERVICE);
  recover_srv_client_ = nh.serviceClient<std_srvs::Trigger>(RECOVER_SERVICE);
}

AdapterSto::~AdapterSto()
{
  hold_srv_client_.shutdown();
  halt_srv_client_.shutdown();
  unhold_srv_client_.shutdown();
  recover_srv_client_.shutdown();
}

void AdapterSto::performStop()
{
  ROS_INFO("Performing STOP due to STO signal");

  std_srvs::Trigger trigger;

  ROS_DEBUG_STREAM("Calling Hold on controller (Service: " << hold_srv_client_.getService() << ")");
  hold_srv_client_.call(trigger);
  ros::Duration(DURATION_BETWEEN_HOLD_AND_DISABLE_MS/1000.0).sleep();

  ROS_DEBUG_STREAM("Calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
  halt_srv_client_.call(trigger);
}

void AdapterSto::unholdController()
{
  if (unhold_srv_client_.exists())
  {
    ROS_DEBUG_STREAM("Calling Unhold (Service: " << unhold_srv_client_.getService() << ")");
    std_srvs::Trigger trigger;
    unhold_srv_client_.call(trigger);
  }
  else
  {
    ROS_WARN_STREAM("Calling Unhold (Service: " << unhold_srv_client_.getService() << ") failed.");
  }
}

void AdapterSto::recoverDrives()
{
  if (recover_srv_client_.exists())
  {
    ROS_DEBUG_STREAM("Calling Recover (Service: " << recover_srv_client_.getService() << ")");
    std_srvs::Trigger trigger;
    recover_srv_client_.call(trigger);
  }
  else
  {
    ROS_WARN_STREAM("Calling Recover (Service: " << recover_srv_client_.getService() << ") failed.");
  }
}

void AdapterSto::updateSto(const bool sto)
{
  // Only react to value changes
  if (sto_.is_initialized() && sto == sto_.value())
  {
    return;
  }
  sto_ = sto;

  if(!sto_.value())
  {
    performStop();
    return;
  }

  unholdController();
  recoverDrives();
}


}  // namespace prbt_hardware_support
