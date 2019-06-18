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

#ifndef OPERATION_MODE_FILTER_H
#define OPERATION_MODE_FILTER_H

#include <vector>

#include <message_filters/simple_filter.h>

#include <prbt_hardware_support/modbus_msg_operation_mode_wrapper.h>
#include <prbt_hardware_support/modbus_msg_operation_mode_wrapper_exception.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/OperationModes.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>

namespace message_filters
{

/**
 * @brief Filter to be applied to modbus messages. Only the messages with changes in the brake test state are passed.
 *        The filter also checks whether the message can be wrapped.
 *
 * \code
 *   message_filters::Subscriber sub<MsgStamped>(nh, TOPIC_NAME, 1);
 *   message_filters::OperationMode filter<MsgStamped>(&sub));
 *
 *   // Register all callback receiving filtered output
 *   filter.registerCallback(&filteredCallback);
 * \endcode
 */
class OperationModeFilter : public SimpleFilter<prbt_hardware_support::ModbusMsgInStamped>
{
public:
  typedef ros::MessageEvent<prbt_hardware_support::ModbusMsgInStamped const> EventType;

  /**
   * @brief Construct the filter and connect to the output of another filter
   */
  template <typename F>
  OperationModeFilter(F &f, const prbt_hardware_support::ModbusApiSpec &api_spec) : api_spec_(api_spec)
  {
    connectInput(f);
  }

  /**
   * @brief Connect to the output of another filter
   */
  template <class F>
  void connectInput(F &f)
  {
    incoming_connection_.disconnect();
    incoming_connection_ = f.registerCallback(
          typename OperationModeFilter::EventCallback(boost::bind(&OperationModeFilter::cb,this, _1)));
  }

private:
  void cb(const EventType &evt)
  {
    try
    {
      prbt_hardware_support::ModbusMsgOperationModeWrapper msg(evt.getMessage(), api_spec_);

      if (msg.getOperationMode() == last_operation_mode_ && !first_call_)
      {
        return;
      }

      last_operation_mode_ = msg.getOperationMode();
      first_call_ = false;
    }
    catch (const prbt_hardware_support::ModbusMsgOperationModeWrapperException &ex)
    {
      ROS_ERROR_STREAM(ex.what() << "\nCan not interpret Modbus message as OperationMode.");
      return;
    }
    catch (const prbt_hardware_support::ModbusMsgWrapperException &ex)
    {
      ROS_ERROR_STREAM(ex.what() << "\nCan not interpret Modbus message as OperationMode.");
      return;
    }

    this->signalMessage(evt);
  }

  Connection incoming_connection_;

  int8_t last_operation_mode_{prbt_hardware_support::OperationModes::UNKNOWN};
  bool first_call_ = true;

  //! Currently valid api_spec (defines modbus register semantic)
  const prbt_hardware_support::ModbusApiSpec api_spec_;
};

} // namespace message_filters

#endif // OPERATION_MODE_FILTER_H
