/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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
#ifndef ROSAPIWRAPPER_H
#define ROSAPIWRAPPER_H

#include <functional>
#include <vector>
#include <string>

#include <ros/node_handle.h>

namespace ros_api_wrapper
{

using std::placeholders::_1;
using std::placeholders::_2;

class RosApiWrapper
{
public:
  template<class MReq, class MRes>
  void addServiceCallbackMember(ros::NodeHandle& nh, const std::string& service_name,
                                std::function<bool(MReq&, MRes&)>& callback);

private:
  std::vector<ros::ServiceServer> services_;
};

template<class MReq, class MRes>
inline void RosApiWrapper::addServiceCallbackMember(ros::NodeHandle& nh, const std::string& service_name,
                                                    std::function<bool(MReq&, MRes&)>& callback)
{
  services_.emplace_back( nh.advertiseService(service_name, callback) );
}


}


#endif // ROSAPIWRAPPER_H
