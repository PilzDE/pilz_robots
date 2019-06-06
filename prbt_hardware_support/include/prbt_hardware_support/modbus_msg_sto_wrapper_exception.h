#ifndef MODBUS_MSG_STO_WRAPPER_EXCEPTION_H
#define MODBUS_MSG_STO_WRAPPER_EXCEPTION_H

#include <string>
#include <stdexcept>

#include <prbt_hardware_support/modbus_msg_wrapper_exception.h>

namespace prbt_hardware_support
{
  /**
   * @brief Expection thrown upon construction of ModbusMsgStoWrapper of the message
   * does not contain the required information.
   *
   */
  class ModbusMsgStoWrapperException : public ModbusMsgWrapperException
  {
    public:
      ModbusMsgStoWrapperException( const std::string& what_arg ):
        ModbusMsgWrapperException(what_arg)
      {
      }
  };

}

#endif // MODBUS_MSG_STO_WRAPPER_EXCEPTION_H
