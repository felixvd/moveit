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

import rospy
import copy

class RobotTrajectoryCommander(object):
    '''
    Manipulation of the /moveit_msgs.msg.RobotTrajectory messages
    '''
    def __init__(self, trajectory):
        self._trajectory = copy.deepcopy(trajectory)
        self._trajectory_duration = self._trajectory.joint_trajectory.points[-1].time_from_start
    
    def append(self, in_trajectory, wait_time_between_trajectories=rospy.Duration.from_sec(0)):
        original_index = len(self._trajectory.joint_trajectory.points)
        original_duration = self._trajectory_duration
        new_points = copy.deepcopy(in_trajectory.joint_trajectory.points)
        # Extend trajectory by new points
        self._trajectory.joint_trajectory.points.extend(new_points)
        # Increase execution time of points by self._trajectory_duration
        for point in self._trajectory.joint_trajectory.points[original_index:-1]:
            point.time_from_start += original_duration + wait_time_between_trajectories
        
        self._trajectory_duration = self._trajectory.joint_trajectory.points[-1].time_from_start

    def get(self):
        return self._trajectory

