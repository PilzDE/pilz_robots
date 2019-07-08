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

#ifndef PRBT_HARDWARE_SUPPORT_SERVICE_CLIENT_FACTORY_H
#define PRBT_HARDWARE_SUPPORT_SERVICE_CLIENT_FACTORY_H

#include <string>

#include <ros/service.h>
#include <prbt_hardware_support/wait_for_service.h>

namespace prbt_hardware_support
{

class ServiceClientFactory
{
public:
  /**
   * @brief Waits for a service to exist and creates a ServiceClient.
   *
   * @param name Name of the service, to be resolved in the node namespace
   * @return The ServiceClient
   */
  template <typename Service>
  inline static ros::ServiceClient create(const std::string &name)
  {
    waitForService(name);
    return ros::service::createClient<Service>(name);
  }
};

}  // namespace prbt_hardware_support

#endif
