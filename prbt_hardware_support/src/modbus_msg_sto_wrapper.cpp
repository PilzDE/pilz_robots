#include <prbt_hardware_support/modbus_msg_sto_wrapper.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>

using namespace prbt_hardware_support;

ModbusMsgStoWrapper::ModbusMsgStoWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw):
  msg_(modbus_msg_raw)
{

  if(!hasVersion(msg_))
  {
    throw ModbusMsgStoWrapperException("Received message does not contain a version.");
  }

  if(!hasSTO(msg_))
  {
    throw ModbusMsgStoWrapperException("Received message does not contain a STO status.");
  }

}