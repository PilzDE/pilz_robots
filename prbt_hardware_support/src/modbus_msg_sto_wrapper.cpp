#include <prbt_hardware_support/modbus_msg_sto_wrapper.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>

using namespace prbt_hardware_support;

ModbusMsgStoWrapper::ModbusMsgStoWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                                         const ModbusApiSpec& api_spec):
  ModbusMsgWrapper(modbus_msg_raw, api_spec)
{
}

void ModbusMsgStoWrapper::checkStructuralIntegrity() const
{
  ModbusMsgWrapper::checkStructuralIntegrity();

  if(!hasSTO())
  {
    throw ModbusMsgStoWrapperException("Received message does not contain a STO status.");
  }
}
