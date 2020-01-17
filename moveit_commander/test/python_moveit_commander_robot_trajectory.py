#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, OMRON SINIC X
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of OMRON SINIC X nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Karoly Istvan Artur

import unittest
import rospy
import rostest
import os
import sys
import copy

import moveit_commander

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

from moveit_commander import RobotCommander, PlanningSceneInterface, RobotTrajectoryCommander


class PythonMoveitCommanderRobotTrajectoryTest(unittest.TestCase):
    PLANNING_GROUP = "b_bot"

    @classmethod
    def setUpClass(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.commander = RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(self.PLANNING_GROUP)

    @classmethod
    def tearDown(self):
        pass

    def plan(self, target):
        self.group.set_joint_value_target(target)
        self.group.set_max_velocity_scaling_factor(1)
        return self.group.plan()

    def set_start_state_to_end_of_trajectory(self, plan):
        """Changes the start state of the move group to the last
        position of the trajectory in the plan parameter. """
        endtime = rospy.Time()
        endtime = plan.joint_trajectory.header.stamp + plan.joint_trajectory.points[-1].time_from_start

        joint_state_at_end_point = JointState()
        joint_state_at_end_point.header.stamp = endtime
        joint_state_at_end_point.name = plan.joint_trajectory.joint_names
        joint_state_at_end_point.position = plan.joint_trajectory.points[-1].positions
        moveit_robot_state_at_end_point = RobotState()
        moveit_robot_state_at_end_point.joint_state = joint_state_at_end_point
        self.group.set_start_state(moveit_robot_state_at_end_point)

    def test_robot_trajectory_commander(self):
        current = self.group.get_current_joint_values()
        modified = self.group.get_current_joint_values()
        modified[0] -= 1.5
        modified[1] -= 1.5
        modified[2] += 1.5

        success1, plan1, time1, err1 = self.plan(modified)
        self.set_start_state_to_end_of_trajectory(plan1)

        success2, plan2, time2, err2 = self.plan(current)
        self.set_start_state_to_end_of_trajectory(plan2)
        self.assertTrue(success1)
        self.assertTrue(success2)

        # create instance
        robot_trajectory = RobotTrajectoryCommander(plan1)

        # check that getting the plan returns the correc trajectory in a different object
        self.assertEqual(robot_trajectory.get(),plan1)
        self.assertIsNot(robot_trajectory.get(),plan1)

        # construct appended plan and check that it does not modify either of the existing plans
        p1 = copy.deepcopy(plan1)
        p2 = copy.deepcopy(plan2)
        robot_trajectory.append(plan2, rospy.Duration.from_sec(3))
        cp1 = copy.deepcopy(plan1)
        cp2 = copy.deepcopy(plan2)

        self.assertEqual(p1,cp1)
        self.assertEqual(p2,cp2)

        #check that the appended plan is not equal to either of the other plans
        self.assertNotEqual(robot_trajectory.get(),plan1)
        self.assertNotEqual(robot_trajectory.get(),plan2)

        # first plan should execute from here
        self.assertTrue(self.group.execute(plan1, True))

        # appended plan should be invalid now
        self.assertFalse(self.group.execute(robot_trajectory.get(), True))

        # second plan should execute from here
        self.assertTrue(self.group.execute(plan2, True))

        rospy.sleep(2)

        # back at 'current' appended plan should execute
        self.assertTrue(self.group.execute(robot_trajectory.get(), True))


if __name__ == '__main__':
    PKGNAME = 'moveit_commander'
    NODENAME = 'python_moveit_commander_robot_trajectory'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, PythonMoveitCommanderRobotTrajectoryTest)

    # suppress cleanup segfault
    os._exit(0)
