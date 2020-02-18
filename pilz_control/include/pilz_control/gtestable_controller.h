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
#ifndef GTESTABLE_CONTROLLER_H
#define GTESTABLE_CONTROLLER_H


#include <controller_interface/controller.h>

namespace gtestable_controller
{

template <class HardwareInterface>
class GtestAbleController: public controller_interface::Controller<HardwareInterface>
{
public:
  GtestAbleController()
    : controller_interface::Controller<HardwareInterface>()
  {

  }

  virtual ~GtestAbleController<HardwareInterface>() {}

  void update(const ros::Time& time, const ros::Duration& period) override {}

};

}

#endif
