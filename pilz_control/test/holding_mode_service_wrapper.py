#!/usr/bin/env python
# Copyright (c) 2020 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy

from std_srvs.srv import Trigger, TriggerRequest


class HoldingModeServiceWrapper:
    """Abstraction around the service call to switch the controller between DEFAULT and HOLDING mode."""

    HOLD_SERVICE_NAME = "hold"
    UNHOLD_SERVICE_NAME = "unhold"
    WAIT_FOR_SERVICE_TIMEOUT_S = 10

    def __init__(self, controller_ns, controller_name):
        hold_service_name = controller_ns + "/" + controller_name + "/" + self.HOLD_SERVICE_NAME
        rospy.wait_for_service(hold_service_name, self.WAIT_FOR_SERVICE_TIMEOUT_S)
        self._hold_srv = rospy.ServiceProxy(hold_service_name, Trigger)

        unhold_service_name = controller_ns + "/" + controller_name + "/" + self.UNHOLD_SERVICE_NAME
        rospy.wait_for_service(unhold_service_name, self.WAIT_FOR_SERVICE_TIMEOUT_S)
        self._unhold_srv = rospy.ServiceProxy(unhold_service_name, Trigger)

    def request_default_mode(self):
        """Switch into DEFAULT mode by sending the respective request.

        :return: True if service request was handled successful, False otherwise.
        """
        req = TriggerRequest()
        resp = self._unhold_srv(req)

        return resp.success

    def request_holding_mode(self):
        """Switch into HOLDING mode by sending the respective request.

        :return: True if service request was handled successful, False otherwise
        """
        req = TriggerRequest()
        resp = self._hold_srv(req)

        return resp.success
