#ifndef PILZ_STO_MODBUS_ADAPTER_NODE_EXCEPTION_H
#define PILZ_STO_MODBUS_ADAPTER_NODE_EXCEPTION_H

#include <string>
#include <stdexcept>

namespace prbt_hardware_support
{

/**
 * @brief Exception used by the PilzStoModbusAdapterNode
 */
class PilzStoModbusAdapterNodeException : public std::runtime_error
{
  public:
    PilzStoModbusAdapterNodeException( const std::string& what_arg );
};

}

#endif // PILZ_STO_MODBUS_ADAPTER_NODE_EXCEPTION_H