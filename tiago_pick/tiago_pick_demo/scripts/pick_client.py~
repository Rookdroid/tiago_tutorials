#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

from random import uniform
import rospy
import time
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose, Twist, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState, GetModelStateResponse, GetModelStateRequest
from tiago_pick_msgs.msg import PickScene, BoxObject
from std_srvs.srv import Empty
import tf
import copy
from tf import transformations

import numpy as np
from std_srvs.srv import Empty

import cv2
from cv_bridge import CvBridge

from moveit_msgs.msg import MoveItErrorCodes

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def reset_world():
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_world_srv = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resp1 = reset_world_srv()
        return
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def move_box_model(box_model):
    model_state = ModelState()
    pose_state = Pose()
    twist_state = Twist()

    pose_state.position = box_model.object_pose.pose.position
    pose_state.orientation = box_model.object_pose.pose.orientation

    twist_state.linear.x = 0.0
    twist_state.linear.y = 0.0
    twist_state.linear.z = 0.0
    twist_state.angular.x = 0.0
    twist_state.angular.y = 0.0
    twist_state.angular.z = 0.0

    model_state.pose = pose_state
    model_state.twist = twist_state
    model_state.model_name = box_model.object_name
    model_state.reference_frame = box_model.object_pose.header.frame_id

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp1 = set_model_srv(model_state)
        print resp1.status_message
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def move_robot_model(pose):
    model_state = ModelState()
    pose_state = Pose()
    twist_state = Twist()

    pose_state = pose

    twist_state.linear.x = 0.0
    twist_state.linear.y = 0.0
    twist_state.linear.z = 0.0
    twist_state.angular.x = 0.0
    twist_state.angular.y = 0.0
    twist_state.angular.z = 0.0

    model_state.pose = pose_state
    model_state.twist = twist_state
    model_state.model_name = "tiago_steel"
    model_state.reference_frame = "world"

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp1 = set_model_srv(model_state)
        print resp1.status_message
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# Create class that keeps tracking of the scene for the pick and place scenario
class Scene(object):
    def __init__(self):
        rospy.loginfo("Start creating the picking scene")
        self.pick_scene = PickScene()
        self.populate_scene_from_param('~scene_objects')
        self.object_height = rospy.get_param('~object_height')
        self.object_width = rospy.get_param('~object_width')
        self.object_depth = rospy.get_param('~object_depth')
        self.surface_height = rospy.get_param('~surface_height')
        self.surface_width = rospy.get_param('~surface_width')
        self.surface_depth = rospy.get_param('~surface_depth')
    def populate_scene_from_param(self, param_name):
        scene_objects = rospy.get_param(param_name)
        for object in scene_objects:
            box_object = BoxObject()
            box_object.object_name = object
            box_object.object_pose.header.frame_id = "world"
            box_object.object_pose.header.stamp = rospy.get_rostime()
            box_object.object_pose.pose.position.x = scene_objects[object][0]
            box_object.object_pose.pose.position.y = scene_objects[object][1]
            box_object.object_pose.pose.position.z = scene_objects[object][2]
            box_object.object_pose.pose.orientation = self.rpy_to_quat_msg(scene_objects[object][3],
                                                                           scene_objects[object][4],
                                                                           scene_objects[object][5])
            self.pick_scene.box_objects_in_scene.append(box_object)

    def set_scene_object_pose(self, object_name, pose):
        for i in range(len(self.pick_scene.box_objects_in_scene)):
            if self.pick_scene.box_objects_in_scene[i].object_name == object_name:
                self.pick_scene.box_objects_in_scene[i].object_pose.header.stamp = rospy.get_rostime()
                self.pick_scene.box_objects_in_scene[i].object_pose.pose.position.x = pose[0]
                self.pick_scene.box_objects_in_scene[i].object_pose.pose.position.y = pose[1]
                self.pick_scene.box_objects_in_scene[i].object_pose.pose.position.z = pose[2]
                self.pick_scene.box_objects_in_scene[i].object_pose.pose.orientation = self.rpy_to_quat_msg(pose[3],
                                                                                                pose[4],
                                                                                                pose[5])
                break
    def get_scene_objects(self):
        return self.pick_scene

    def is_scene_pos_occupied(self, test_box):
        for i in range(len(self.pick_scene.box_objects_in_scene)):
            if self.pick_scene.box_objects_in_scene[i].object_name == test_box.object_name:
                continue
            occupied_space = PoseStamped()
            occupied_space = self.pick_scene.box_objects_in_scene[i].object_pose
            if (abs(occupied_space.pose.position.x - test_box.object_pose.pose.position.x) < (self.object_width * 1.5)) or \
                    (abs(occupied_space.pose.position.y - test_box.object_pose.pose.position.y) < (self.object_depth * 1.5)):
                return True
        return False

    def randomize_object_poses(self):
        for i in range(len(self.pick_scene.box_objects_in_scene)):
            valid_pose = False
            while not valid_pose:
                test_box = BoxObject()
                test_box.object_name = self.pick_scene.box_objects_in_scene[i].object_name
                test_box.object_pose.header.frame_id = "world"
                test_box.object_pose.header.stamp = rospy.get_rostime()
                test_box.object_pose.pose.position.x = uniform(0.47, 1.0)
                test_box.object_pose.pose.position.y = uniform(-0.33, 0.34)
                test_box.object_pose.pose.position.z = 0.8645
                test_box.object_pose.pose.orientation = self.rpy_to_quat_msg(0.0, 0.0, 0.0)#uniform(0.0, 3.14))
                if not self.is_scene_pos_occupied(test_box):
                    valid_pose = True
            move_box_model(test_box)
            self.pick_scene.box_objects_in_scene[i] = test_box


    def rpy_to_quat_msg(self,r,p,y):
        quaternion = tf.transformations.quaternion_from_euler(r, p, y)
        return Quaternion(*quaternion)

class SphericalService(object):
    def __init__(self):
        rospy.loginfo("Starting Spherical Grab Service")
        self.pick_type = PickAruco()
        rospy.loginfo("Finished SphericalService constructor")
        self.place_gui = rospy.Service("/place_gui", Empty, self.start_aruco_place)
        self.pick_gui = rospy.Service("/pick_gui", Empty, self.start_aruco_pick)
    def start_aruco_pick(self, req):
        self.pick_type.pick_aruco("pick")
        return {}

    def start_aruco_place(self, req):
        self.pick_type.pick_aruco("place")
        return {}


class PickAruco(object):
    def __init__(self):
        self.first_go = True
        #reset_world()
        self.pick_scene = Scene()

        rospy.loginfo("Initalizing...")
        self.bridge = CvBridge()
        self.tf_l = tf.TransformListener()
        rospy.loginfo("Waiting for /pickup_pose AS...")
        self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        time.sleep(1.0)
        if not self.pick_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /pickup_pose AS")
            exit()
        rospy.loginfo("Waiting for /place_pose AS...")
        self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)

        self.place_as.wait_for_server()

        rospy.loginfo("Setting publishers to torso and head controller...")
        self.torso_cmd = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1)
        self.detected_pose_pub = rospy.Publisher('/detected_aruco_pose',
                                                 PoseStamped,
                                                 queue_size=1,
                                                 latch=True)
        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel",Twist,queue_size=10)

        rospy.loginfo("Waiting for '/play_motion' AS...")
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        if not self.play_m_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()
        rospy.loginfo("Connected!")
        rospy.sleep(1.0)
        rospy.loginfo("Done initializing PickAruco.")

    def rpy_to_quat_msg(self,r,p,y):
        quaternion = tf.transformations.quaternion_from_euler(r, p, y)
        return Quaternion(*quaternion)

    def get_model_state(self, model_name, reference_model_name):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1 = GetModelStateResponse()
            resp1 = get_model_srv(model_name,reference_model_name)
            return resp1.pose
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def pick_aruco(self, string_operation):
        if self.first_go:
            self.prepare_robot()

        self.pick_scene.randomize_object_poses()
        rospy.sleep(2.0)
        pick_g = PickUpPoseGoal()
        if string_operation == "pick":
            pick_order = sorted(self.pick_scene.pick_scene.box_objects_in_scene, key=lambda object: object.object_pose.pose.position.y, reverse=False)
            print pick_order

            for idx in range(len(self.pick_scene.pick_scene.box_objects_in_scene)):
                object = pick_order[idx]
                pick_g.object_to_pick = object.object_name
                pick_g.pick_scene.box_objects_in_scene = self.pick_scene.pick_scene.box_objects_in_scene
                pick_g.object_pose.header.frame_id = 'base_footprint'
                pick_g.object_pose.pose.orientation.w = 1.0
                self.pick_as.send_goal_and_wait(pick_g)
                rospy.loginfo("Done!")

                result = self.pick_as.get_result()
                if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                    rospy.logerr("Failed to pick, not trying further")
                    return

                # Move torso to its maximum height
                self.lift_torso()

                # Raise arm
                rospy.loginfo("Moving arm to a safe pose")
                pmg = PlayMotionGoal()
                pmg.motion_name = 'pick_final_pose'
                pmg.skip_planning = False
                self.play_m_as.send_goal_and_wait(pmg)
                rospy.loginfo("Raise object done.")

                # Rotate to kit
                rotation_speed = 0.628            #spin ot 0.3 rads/s
                twist_command = Twist()
                twist_command.linear.x = 0.0
                twist_command.linear.y = 0.0
                twist_command.linear.z = 0.0
                twist_command.angular.x = 0.0
                twist_command.angular.y = 0.0
                twist_command.angular.z = rotation_speed

                cycles_per_second = 3
                rate = rospy.Rate(cycles_per_second)
                number_of_cycles = 3.14/rotation_speed*cycles_per_second
                current_cycle = 0
                rospy.loginfo("Spin to kit tray")
                while (not rospy.is_shutdown()) and (current_cycle < number_of_cycles + 2):
                    self.cmd_vel_pub.publish(twist_command)
                    rate.sleep()
                    current_cycle += 1
                rospy.sleep(2.0)

                #Place object
                place_pose = PoseStamped()
                place_pose.header.frame_id = 'base_footprint'
                place_pose.header.stamp = rospy.get_rostime()
                place_pose.pose = self.get_model_state("kit_tray","base_footprint")
                place_pose.pose.position.z += 0.01 + 0.125
                place_pose.pose.position.y = place_pose.pose.position.y-0.25 + 0.1*idx
                rospy.loginfo(place_pose)
                place_pose.pose.orientation.x = 0.0
                place_pose.pose.orientation.y = 0.0
                place_pose.pose.orientation.z = 0.0
                place_pose.pose.orientation.w = 1.0
                rospy.loginfo("Place object on tray")
                pick_g.object_pose = place_pose
                pick_g.object_to_pick = object.object_name  #Action naming is currently wrong. It should be called object to place
                pick_g.pick_scene = self.pick_scene.pick_scene   #objects to be removed from scene
                self.place_as.send_goal_and_wait(pick_g)
                rospy.loginfo("Done!")

                #Rotate back
                current_cycle = 0
                while (not rospy.is_shutdown()) and (current_cycle < number_of_cycles + 2):
                    self.cmd_vel_pub.publish(twist_command)
                    rate.sleep()
                    current_cycle += 1



    def lift_torso(self):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)

    def lower_head(self):
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")

    def prepare_robot(self):
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pregrasp'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")

        self.lower_head()
        self.first_go = False
        rospy.loginfo("Robot prepared.")


if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    sphere = SphericalService()
    rospy.spin()
