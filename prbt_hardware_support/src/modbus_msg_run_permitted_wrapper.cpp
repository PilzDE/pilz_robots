#include <prbt_hardware_support/modbus_msg_run_permitted_wrapper.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>

using namespace prbt_hardware_support;

ModbusMsgRunPermittedWrapper::ModbusMsgRunPermittedWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                                         const ModbusApiSpec& api_spec):
  ModbusMsgWrapper(modbus_msg_raw, api_spec)
{
}

void ModbusMsgRunPermittedWrapper::checkStructuralIntegrity() const
{
  ModbusMsgWrapper::checkStructuralIntegrity();

  if(!hasRUN_PERMITTED())
  {
    throw ModbusMsgRunPermittedWrapperException("Received message does not contain a RUN_PERMITTED status.");
  }
}
