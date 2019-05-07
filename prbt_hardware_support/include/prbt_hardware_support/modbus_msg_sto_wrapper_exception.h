#ifndef MODBUS_MSG_STO_WRAPPER_EXCEPTION_H
#define MODBUS_MSG_STO_WRAPPER_EXCEPTION_H

#include <string>
#include <stdexcept>

namespace prbt_hardware_support
{
  /**
   * @brief Expection thrown upon construction of ModbusMsgStoWrapper of the message
   * does not contain the required information.
   *
   */
  class ModbusMsgStoWrapperException : public std::runtime_error
  {
    public:
      ModbusMsgStoWrapperException( const std::string& what_arg ):
        std::runtime_error(what_arg)
      {
      }
  };

}

#endif // MODBUS_MSG_STO_WRAPPER_EXCEPTION_H
