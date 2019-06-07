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

#ifndef BRAKE_TEST_FILTER_H
#define BRAKE_TEST_FILTER_H

#include <vector>

#include <message_filters/simple_filter.h>

#include <prbt_hardware_support/modbus_msg_brake_test_wrapper.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper_exception.h>
#include <prbt_hardware_support/modbus_api_spec.h>

namespace message_filters
{

/**
 * @brief Filter to be applied to modbus messages. Only the messages with changes in the brake test state are passed.
 *        The filter also checks whether the message can be wrapped.
 *        Disconnect messages are not passed along.
 *
 * \code
 *   message_filters::Subscriber sub<MsgStamped>(nh, TOPIC_NAME, 1);
 *   message_filters::BrakeTestFilter filter<MsgStamped>(&sub));
 *
 *   // Register all callback receiving filtered output
 *   filter.registerCallback(&filteredCallback);
 * \endcode
 */
template <typename M>
class BrakeTestFilter : public SimpleFilter<M>
{
public:
  typedef boost::shared_ptr<M const> MConstPtr;
  typedef ros::MessageEvent<M const> EventType;

  /**
   * @brief Construct the filter and connect to the output of another filter
   */
  template <typename F>
  BrakeTestFilter(F &f, const prbt_hardware_support::ModbusApiSpec &api_spec) : api_spec_(api_spec)
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
    incoming_connection_ = f.registerCallback(typename BrakeTestFilter<M>::EventCallback(boost::bind(&BrakeTestFilter::cb,
                                                                                                     this, _1)));
  }

private:
  void cb(const EventType &evt)
  {
    try
    {
      prbt_hardware_support::ModbusMsgBrakeTestWrapper brakeTestWrapper(evt.getMessage(), api_spec_);

      if (!brakeTestWrapper.isDisconnect())
      {
        if (brakeTestWrapper.isBrakeTestRequired() == last_brake_test_required_ && !first_call_)
        {
          return;
        }

        last_brake_test_required_ = brakeTestWrapper.isBrakeTestRequired();
        first_call_ = false;
      }
    }
    catch (const prbt_hardware_support::ModbusMsgWrapperException &ex)
    {
      ROS_ERROR_STREAM(ex.what() << "\nCan not interpret Modbus message.");
      return;
    }
    catch (const prbt_hardware_support::ModbusMsgBrakeTestWrapperException &ex)
    {
      ROS_ERROR_STREAM(ex.what() << "\nCan not interpret Modbus message as ModbusMsgBrakeTest.");
      return;
    }

    this->signalMessage(evt);
  }

  Connection incoming_connection_;

  bool last_brake_test_required_;
  bool first_call_ = true;

  //! Currently valid api_spec (defines modbus register semantic)
  const prbt_hardware_support::ModbusApiSpec api_spec_;
};

} // namespace message_filters

#endif // BRAKE_TEST_FILTER_H
