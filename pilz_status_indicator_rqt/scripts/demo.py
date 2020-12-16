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

import random

import rospy
from numpy import arange
from pilz_msgs.msg import OperationModes
from std_msgs.msg import Bool, Float64

from pilz_status_indicator_rqt.status_indicator import (TOPIC_STATE_HW,
                                                        TOPIC_STATE_ROS,
                                                        TOPIC_OPERATION_MODE,
                                                        TOPIC_SPEED_OVERRIDE)

if __name__ == "__main__":
    pubs = [
        rospy.Publisher(TOPIC_STATE_HW, Bool, queue_size=1),
        rospy.Publisher(TOPIC_STATE_ROS, Bool, queue_size=1),
        rospy.Publisher(TOPIC_OPERATION_MODE, OperationModes, queue_size=1),
        rospy.Publisher(TOPIC_SPEED_OVERRIDE, Float64, queue_size=1)
    ]
    states = [
        [True, False],
        [True, False],
        [
            OperationModes.AUTO,
            OperationModes.T1,
            OperationModes.T2
        ],
        list(arange(0, 1, .07))
    ]
    constructors = [
        lambda x: Bool(x),
        lambda x: Bool(x),
        lambda x: OperationModes(time_stamp=None, value=x),
        lambda x: Float64(x)
    ]

    rospy.init_node("status_indicator_demo")
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        to_change = random.randint(0, len(states)-1)
        pubs[to_change].publish(
            constructors[to_change](random.choice(states[to_change]))
        )
        rate.sleep()
