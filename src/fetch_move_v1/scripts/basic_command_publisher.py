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
from geometry_msgs.msg import PointStamped, Point, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# DEMO_P1 = True
DEMO_P2 = False


head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0, 0.0]
head_joint_velocities = [0.0, 0.0]
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

prev_actor_position = Point(x=-1000.0,y=0.0,z=0.0)

soundhandle = SoundClient()
voice = 'voice_kal_diphone'
volume = 1.0

# base_pub = rospy.Publisher('base_controller/command', Twist, queue_size=10)

MIN_LIN_VEL = -1.0
MAX_LIN_VEL = 1.0
MIN_ANG_VEL = -0.5
MAX_ANG_VEL = 0.5

# def Dist3D(p1, p2):
#     dist = 0.0
#     for i in range(3):
#         dist += ((p2[i]-p1[i])**2)
#     return dist
class MapGoalPos:
    def __init__(self, name, x, y, rz, rw):
        self.name = name
        self.x = x
        self.y = y
        self.rz = rz
        self.rw = rw

# For Map CRL_G_Nov16
# You will need to update these if you generate a new map.
# Nov16 Map
# G1 = MapGoalPos("G1", 2.619, -4.452, 0.644, 0.764) # Lab Entrance
# G2 = MapGoalPos("G2", 5.703, -4.177, 0.913, 0.406) # Office Entrance
# G3 = MapGoalPos("G3", 2.298, 0.065, -0.573, 0.818) # Home/Start Pos

# Nov16 Map
G1 = MapGoalPos("G1", 0.100, 0.645, -0.499, 0.866) # Lab Entrance
G2 = MapGoalPos("G2", -1.371, -1.858, 0.057, 0.998) # Office Entrance
G3 = MapGoalPos("G3", 3.871, -2.070, 0.998, 0.049) # Home/Start Pos

MapGoals = {"G1": G1, "G2": G2, "G3": G3}

def GetNewGoal(curr_goal, direction):
    if direction=="left":
        if curr_goal == "G1":
            return "G2"
        elif curr_goal == "G2":
            return "G3"
        elif curr_goal == "G3":
            return "G1"
        else:
            raise Exception("Invalid goal. Pick from [G1,G2,G3]")
    elif direction=="right":
        if curr_goal == "G1":
            return "G3"
        elif curr_goal == "G2":
            return "G1"
        elif curr_goal == "G3":
            return "G2"
        else:
            raise Exception("Invalid goal. Pick from [G1,G2,G3]")
    else:
        raise Exception("Invalid direction. Pick from 'left' and 'right'.")

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


def MoveHead(goal_time, move_type="pos", pan = 0.0, tilt = 0.0):
    """
    move_type: "pos" or "vel"
    goal_time: Seconds to reach specified goal.
               (Be careful when setting it low values i.e. < 1.0 sec, 
                you might trip breakers if the movements are too fast!)
                To reset them, you may need to run 'sudo service robot stop && start
    """
    # global IS_MOVING
    # IS_MOVING = True

    trajectory = JointTrajectory()
    trajectory.joint_names = head_joint_names

    trajectory.points.append(JointTrajectoryPoint())
    if move_type == "pos":
        head_joint_positions[0] = pan
        head_joint_positions[1] = tilt
        trajectory.points[0].positions = head_joint_positions
        trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    elif move_type == "vel":
        head_joint_velocities[0] = pan
        head_joint_velocities[1] = tilt
        trajectory.points[0].positions = [0.0] * len(head_joint_positions)
        trajectory.points[0].velocities = head_joint_velocities
    else:
        raise Exception('Invalid move_type for MoveHead(). Only "pos" and "vel" allowed.')

    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(goal_time)

    rospy.loginfo("[ head pan ] %f, [ head tilt ] %f", head_joint_positions[0], head_joint_positions[1])

    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory = trajectory
    head_goal.goal_time_tolerance = rospy.Duration(0.0)

    # rospy.loginfo("Setting positions...")
    head_client.send_goal(head_goal)
    head_client.wait_for_result(rospy.Duration(goal_time + 2.0))

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, z, w, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        # move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        # move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.pose.orientation.z = z
        move_goal.target_pose.pose.orientation.w = w
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()


class CommandHandler:
    def __init__(self):
        rospy.init_node("basic_command_publisher")
        self.body_pose_sub = rospy.Subscriber('body25_pose_data', Float32MultiArray, self.body_pose_callback)
        self.human_pos_sub =  rospy.Subscriber('human_pos', PointStamped, self.human_position_callback)
        self.teleop_command_sub = rospy.Subscriber('pose_teleop_commands', Int32MultiArray, self.teleop_command_callback)
        
        self.base_command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0

        self.actor_position = Point()
        self.actor_position.x = 0.0
        self.actor_position.y = 0.0
        self.actor_position.z = 0.0

        self.pose_teleop_command = Int32MultiArray()
        self.pose_teleop_command.data = [0,0]
        self.previous_pose_teleop_command = [0,0]
        
        self.current_map_goal = "G3"

        self.current_state = 0
        self.states = {0: "Initializeing", 
                       1: "Waiting",
                       2: "Searching",
                       3: "Moving Forward",
                       4: "Moving Backward",
                       5: "Moving to Goal",
                       6: "Making a Circle",
                       7: "Looking Left",
                       8: "Looking Right"
                      }

    def publish_base_command(self):
        for i in range(5):
            self.base_command_pub.publish(self.twist_msg)
        rospy.loginfo("Publishing cmd_vel commands...")
        # self.stop_moving()

    def stop_moving(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        rospy.loginfo("Stopping...")

    def move_forward(self):
        if self.previous_pose_teleop_command[0] == 1:
            self.twist_msg.linear.x += 0.05
            self.twist_msg.linear.x = min(self.twist_msg.linear.x, 0.4)
        else:
            self.twist_msg.linear.x = 0.1
        self.twist_msg.angular.z = 0.0
        rospy.loginfo("Moving forward...")

    def move_backward(self):
        self.twist_msg.linear.x = -0.2
        self.twist_msg.angular.z = 0.0
        rospy.loginfo("Moving backward...")

    def make_circle(self):
        self.twist_msg.linear.x = 0.3
        self.twist_msg.angular.z = 1.0
        rospy.loginfo("Making circle...")

    # def move_right(self):
    #     self.twist_msg.linear.x = 0.0
    #     self.twist_msg.angular.z = 0.3*-1.0
    #     rospy.loginfo("Moving right...")
    #     pass

    # def move_left():
    #     #TODO
    #     pass

    def rotate_body(self, direction, speed=0.3):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = speed * (-1.0 if direction=="right" else 1.0)
        rospy.loginfo(f"Rotating {direction}...")

    def rotate_body_fixed_head(self, direction, speed=0.3):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = speed * (-1.0 if direction=="right" else 1.0)
        rospy.loginfo(f"Rotating {direction}...")
        MoveHead(0.1, 0.05*(-1.0 if direction=="right" else 1.0), 0.0)

    def teleop_command_callback(self, data):
        # rospy.loginfo(f"I heard Pose Teleop Command, [{data.data[0], data.data[1]}]")
        self.pose_teleop_command.data = data
    
    def human_position_callback(self, data):
        # global actor_position
        global curr_pan_goal
        # rospy.loginfo(rospy.get_caller_id() + "I heard human position data")

        y_speed_limit = 0.1
        z_speed_limit = 0.1
        pan_speed = 0.00001

        if prev_actor_position.x != -1000.0:
            # We won't track fast moving objects
            if abs(prev_actor_position.y-data.point.y) < y_speed_limit and abs(prev_actor_position.z-data.point.z) < z_speed_limit:
                actor_position = data.point
                # head_action.look_at(2.0, actor_position.y, 0.0, 'head_camera_rgb_frame')
                if actor_position.y > 0.0:
                    curr_pan_goal += pan_speed*abs(actor_position.y)
                elif actor_position.y < 0.0:
                    curr_pan_goal -= pan_speed*abs(actor_position.y)
                # MoveHead(0.001, curr_pan_goal, 0.0)
            
        prev_actor_position.x = data.point.x
        prev_actor_position.y = data.point.y
        prev_actor_position.z = data.point.z
        # head_action.look_at(actor_position.x, actor_position.y, actor_position.z, 'odom')

    def body_pose_callback(self, data):
        global human_Xpos
        global curr_pan_goal
        global prev_time
        global curr_error, last_error, cum_error, rate_error
        # rospy.loginfo(rospy.get_caller_id() + "I heard pose data")
        
        # Simply putting this here will cause buffering issues.
        # soundhandle.say("I see a human", voice, volume)

        # Get Keypoints data
        RShoulder_kp = [data.data[4], data.data[5]]
        Neck_kp = [data.data[2], data.data[3]]
        LShoulder_kp = [data.data[10], data.data[11]]
        MidHip_kp = [data.data[16], data.data[17]]
        
        # Calculate Human X Position (Setpoint)
        if LShoulder_kp[0] > 0.0 and RShoulder_kp[0] > 0.0:
            if Neck_kp[0] > 0.0:
                human_Xpos = (LShoulder_kp[0] + Neck_kp[0] + RShoulder_kp[0])// 3
            else:
                human_Xpos = (LShoulder_kp[0] + RShoulder_kp[0])// 3
        elif Neck_kp[0] > 0.0:
            if MidHip_kp[0] > 0.0:
                human_Xpos = (Neck_kp[0]+MidHip_kp[0])//2
            else:
                human_Xpos = Neck_kp[0] 
        else:  # If robot can't see shoulders, only hips visible
            if MidHip_kp[0] > 0.0:
                human_Xpos = MidHip_kp[0]
            else:
                human_Xpos = 320# Assume human is in the center.
        
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
        # rospy.loginfo(f"goal pos: {curr_pan_goal}")

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
        # MoveHead(0.0001, curr_pan_goal, 0.0)
        
            # prev_time = time.time()
        # return data.data



if __name__ == "__main__":
    
    

    # ALWAYS CHECK YOUR CODE IN SIMULATION BEFORE RUNNING IT ON THE REAL ROBOT.
    # Check robot serial number, we never want to run this on a real robot!
    # if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
    #     rospy.logerr("This script should not be run on a real robot")
    #     sys.exit(-1)
    CH = CommandHandler()

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Hello--------------------------")

    head_action = PointHeadClient()
    if DEMO_P2:
        move_base = MoveBaseClient()
    
    # Put say calls before Move()

    # soundhandle.say("Moving to home position", voice, volume)
    soundhandle.say(CH.states[CH.current_state], voice, volume)
    MoveHead(1.0, pan=0.0, tilt=0.0)
    rospy.sleep(1.0)
    
    # soundhandle.say("Looking Right", voice, volume)
    # MoveHead(1.0, -1.0, 0.0)
    # rospy.sleep(1.0)
    
    # head_action.look_at(1.0, 1.0, 0.0, 'head_camera_rgb_frame')

    # soundhandle.say("Looking Left", voice, volume)
    # MoveHead(1.0, 1.0, 0.0)
    # rospy.sleep(1.0)

    # soundhandle.say("Moving to home position", voice, volume)
    # MoveHead(1.0, 0.0, 0.0)
    # rospy.sleep(1.0)

    curr_time = rospy.Time.now()
    prev_time = 0.0
    prev_speech_time = 0.0
    speech_delay = 4.0
    delay = 0.3

    make_circle_timer = 10.0
    make_circle_start_time = 0.0
    

    # rospy.Time.now()
    try:
        while not rospy.is_shutdown():
            curr_time = rospy.Time.now().to_time()
            # rospy.loginfo("--[ RUNNING ]--")
            if CH.current_state == 6 and (curr_time - make_circle_start_time) < make_circle_timer:
                CH.publish_base_command()
                pass
            elif (curr_time - prev_time) > delay:
                # SEARCH
                # rospy.loginfo("--[ SEARCHING ]--")
                if human_Xpos > 380.0:
                    speed = 0.7*(abs(human_Xpos-320.0)/320.0)
                    CH.rotate_body("right", speed)
                    CH.current_state = 2

                    # soundhandle.say("Rotating right", voice, volume)
                    rospy.loginfo(f"--[ Rotating RIGHT {speed}]--")
                elif human_Xpos < 240.0:
                    speed = 0.7*(abs(human_Xpos-320.0)/320.0)
                    CH.rotate_body("left", speed)
                    CH.current_state = 2
                    # soundhandle.say("Rotating left", voice, volume)
                    rospy.loginfo(f"--[ Rotating LEFT {speed}]--")
                else:
                    # TELEOP BASED ON POSE
                    # TODO : 
                    # - Add counters for each pose so that the robot moves only after getting
                    #   n consecutive frames with a pose.(say 4)
                    # - Improve Human detection by filtering based on pose.(or height of neck kp)
                    #   Maybe say "too far away if person is too far away"
                    # - 
                    # CH.stop_moving()
                    if type(CH.pose_teleop_command.data) is not list:
                        if CH.pose_teleop_command.data.data[0] == 1: # Left_arm_bent
                            CH.move_forward()
                            CH.current_state = 3
                            CH.previous_pose_teleop_command[0] = 1

                        elif CH.pose_teleop_command.data.data[0] == -1: # Right_arm_bent
                            CH.move_backward()
                            CH.current_state = 4
                            CH.previous_pose_teleop_command[0] = -1

                        # elif CH.pose_teleop_command.data.data[0] == 2 and CH.pose_teleop_command.data.data[1] == 2:
                        #     CH.make_circle()
                        #     make_circle_start_time = curr_time
                        #     CH.current_state = 6
                        #     CH.previous_pose_teleop_command[0] = 2

                        elif DEMO_P2:
                            if CH.pose_teleop_command.data.data[1] == 1: # Point_left
                                new_goal = MapGoals[GetNewGoal(CH.current_map_goal, "left")]
                                CH.current_state = 5
                                CH.current_map_goal = new_goal.name
                                move_base.goto(new_goal.x, new_goal.y, new_goal.rz, new_goal.rw)
                                MoveHead(1.0, pan=0.0, tilt=0.0)

                            elif CH.pose_teleop_command.data.data[1] == -1: # Point_right
                                new_goal = MapGoals[GetNewGoal(CH.current_map_goal, "right")]
                                CH.current_state = 5
                                CH.current_map_goal = new_goal.name
                                move_base.goto(new_goal.x, new_goal.y, new_goal.rz, new_goal.rw)
                                MoveHead(1.0, pan=0.0, tilt=0.0)

                        else:
                            CH.stop_moving()
                            CH.current_state = 1
                            CH.previous_pose_teleop_command[0] = 0
                
                prev_time = curr_time
            
                CH.publish_base_command()
            
            if (curr_time - prev_speech_time) > speech_delay:
                prev_speech_time = curr_time
                message = CH.states[CH.current_state]
                if CH.current_state == 5:
                    message += f" {CH.current_map_goal}"
                soundhandle.say(message, voice, volume)
        

    except KeyboardInterrupt:
        rospy.signal_shutdown()


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
