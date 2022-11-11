#!/usr/bin/env python3
# If you get an error where this node crashes, it's probably becuase the above line points to a python version
# that doesn't exist on your PC. Change it to 'python3' or wherever your python is installed.
# You can see this when you run 'catkin_make', in the first couple of lines.

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import sys

import rospy
import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              PointHeadAction,
                              PointHeadGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import time
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0, 0.0]
head_pan_MAX = 1.0
head_pan_inc = 0.9
head_tilt_MAX = 0.8
head_tilt_inc = 0.2

curr_pan_goal = 0.0
curr_tilt_goal = 0.0

human_Xpos = 300.0
IS_MOVING = False

Kp = 0.02
Ki = 0.5
Kd = 0.2
curr_error = 0.0
last_error = 0.0
cum_error = 0.0
rate_error = 0.0

prev_time = 0.0
check_interval = 0.5 # sec

actor_position = [0.0, 0.0, 2.0]

soundhandle = SoundClient()
voice = 'voice_kal_diphone'
volume = 1.0

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()


def MoveHead(goal_time, pan = 0.0, tilt = 0.0):
    """
    goal_time: Seconds to reach specified goal.
               (Be careful when setting it low values i.e. < 1.0 sec, 
                you might trip breakers if the movements are too fast)
    """
    # global IS_MOVING
    # IS_MOVING = True
    head_joint_positions[0] = pan
    head_joint_positions[1] = tilt

    trajectory = JointTrajectory()
    trajectory.joint_names = head_joint_names

    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = head_joint_positions
    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(goal_time)

    rospy.loginfo("[ head pan ] %f, [ head tilt ] %f", head_joint_positions[0], head_joint_positions[1])

    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory = trajectory
    head_goal.goal_time_tolerance = rospy.Duration(0.0)

    # rospy.loginfo("Setting positions...")
    head_client.send_goal(head_goal)
    head_client.wait_for_result(rospy.Duration(goal_time + 2.0))

def position_callback(data):
    global actor_position
    actor_position = data

def callback(data):
    global human_Xpos
    global curr_pan_goal
    global prev_time
    global curr_error, last_error, cum_error, rate_error
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    # Get Keypoints data
    LShoulder_kp = [data.data[0], data.data[1]]
    Neck_kp = [data.data[2], data.data[3]]
    RShoulder_kp = [data.data[4], data.data[5]]
    
    # Calculate Human X Position (Setpoint)
    human_Xpos = (LShoulder_kp[0] + Neck_kp[0] + RShoulder_kp[0])// 3
    # rospy.loginfo(f"[[ human_pos: {human_Xpos} ]]")
    # head_action.look_at(0.0, 0.0, 0.0, 'base_link/torso_lift_link/head_pan_link/head_tilt_link/head_camera_link/head_camera_rgb_frame/human_actor')
    # head_action.look_at(actor_position[0], actor_position[1], actor_position[2], 'head_camera_rgb_frame')
    # curr_time = time.time()
    # dt = (curr_time - prev_time)
    # curr_error = human_Xpos - 320.0
    # cum_error += (curr_error*dt)
    # rate_error = (curr_error - last_error)/dt

    # curr_pan_goal = (Kp*curr_error)+(Kd*rate_error)
    curr_pan_goal = (human_Xpos-320.0)/320.0
    curr_pan_goal = max(-1.2, min(curr_pan_goal, 1.2))
    rospy.loginfo(f"goal pos: {curr_pan_goal}")

    # last_error = curr_error
    # prev_time = curr_time

    # MoveHead(0.001, curr_pan_goal, 0.0)
    
    # rospy.loginfo(f"dt: {(time.time())}")
    # if (time.time()-prev_time) > check_interval:
        # p_gain = 0.05 + 0.4*abs(human_Xpos - 320.0)/320.0
        # rospy.loginfo(f"gain: {p_gain}")
        # if human_Xpos < 290.0:
        #     rospy.loginfo("Moving head left...")
        #     curr_pan_goal = head_joint_positions[0]+p_gain
        #     # IS_MOVING = False

        # elif human_Xpos > 330.0:
        #     rospy.loginfo("Moving head right...")
        #     curr_pan_goal = head_joint_positions[0]-p_gain
        #     # IS_MOVING = False
        # curr_pan_goal = (human_Xpos-320)/320.0 - head_joint_positions[0]
        # if p_gain != 0.0:
    MoveHead(0.0001, curr_pan_goal, 0.0)
    
        # prev_time = time.time()
    # return data.data
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    

    # rospy.Subscriber('pose_data', Int32MultiArray, callback)
    rospy.Subscriber('pose_data', Int32MultiArray, callback)
    rospy.Subscriber('actor_position', Float32MultiArray, position_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    rospy.loginfo("[ inside listener ]")


if __name__ == "__main__":
    
    rospy.init_node("basic_command_publisher")

    # ALWAYS CHECK YOUR CODE IN SIMULATION BEFORE RUNNING IT ON THE REAL ROBOT.
    # Check robot serial number, we never want to run this on a real robot!
    # if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
    #     rospy.logerr("This script should not be run on a real robot")
    #     sys.exit(-1)

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")

    head_action = PointHeadClient()
    # WE WON'T BE USING THESE
    # rospy.loginfo("Waiting for arm_controller...")
    # arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    # arm_client.wait_for_server()
    # rospy.loginfo("...connected.")

    # rospy.loginfo("Waiting for gripper_controller...")
    # gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    # gripper_client.wait_for_server()
    # rospy.loginfo("...connected.")
    
    soundhandle.say("Moving to home position", voice, volume)
    MoveHead(1.0, 0.0, 0.0)

    listener()

    # MoveHead is absolute i.e. move to specified absolute angle.
    # MoveHead(1.0, 0.0, 0.0)
    # rospy.loginfo("Move 1 done")
    # MoveHead(1.0, 1.0, 0.0)
    # rospy.loginfo("Move 2 done")
    # MoveHead(1.0, 1.2, 0.0)
    # rospy.loginfo("Move 3 done")
    # MoveHead(1.0, -1.2, 0.8)
    # rospy.loginfo("Move 4 done")
    # MoveHead(1.0, 1.2, 0.0)
    # rospy.loginfo("Move 5 done")
        
    
    # while not rospy.is_shutdown():
        

    #     head_joint_positions[0] += head_pan_inc

    #     if head_joint_positions[0] >= head_pan_MAX:
    #         head_pan_inc *= -1.0
    #         head_joint_positions[1] += head_tilt_inc
    #     elif head_joint_positions[0] <= -head_pan_MAX:
    #         head_pan_inc *= -1.0
    #         head_joint_positions[1] += head_tilt_inc
        
    #     if head_joint_positions[1] >= head_tilt_MAX:
    #         head_tilt_inc *= -1.0
    #     elif head_joint_positions[1] <= -head_tilt_MAX:
    #         head_tilt_inc *= -1.0
    
    rospy.loginfo("...done")