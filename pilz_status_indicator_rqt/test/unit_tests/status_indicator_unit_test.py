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

import unittest
from mock import patch, ANY

import rospy

from qt_gui.plugin import Plugin

from pilz_status_indicator_rqt.status_indicator import PilzStatusIndicatorRqt


class MockContext():
    def __init__(self):
        pass


class TestStatusIndicator(unittest.TestCase):
    """
    TODO
    """

    def fake_init(obj):
        """TODO"""
        pass

    @patch.object(Plugin, '__init__')
    def test_bootup(self, mockinit):
        mc = MockContext()
        mockinit.assert_called_with(ANY)
        psi = PilzStatusIndicatorRqt(mc)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_status_indicator')
    rostest.rosrun('pilz_status_indicator_rqt',
                   'test_status_indicator', TestStatusIndicator)
