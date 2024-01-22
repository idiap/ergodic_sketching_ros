# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose
import tf2_ros

import time

import numpy as np

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s

class Keypoint:

    def __init__(self,server,name,base_frame,pos,orn):
        self.__uid = name
        self.__marker = InteractiveMarker()
        self.__marker.header.frame_id = base_frame
        self.__marker.name = self.__uid

        self.__precision_matrix = np.diag([1,1,1,.1,.1,.1,0])

        self.__pos = pos
        self.__orn = orn

        self.__marker.pose.position.x = pos[0]
        self.__marker.pose.position.y = pos[1]
        self.__marker.pose.position.z = pos[2]

        self.__marker.pose.orientation.w = orn[0]
        self.__marker.pose.orientation.x = orn[1]
        self.__marker.pose.orientation.y = orn[2]
        self.__marker.pose.orientation.z = orn[3]

        self.__marker.scale = 0.2

        self._marker_pose_pub = rospy.Publisher(f"~{name}_pose",Pose,queue_size=10)

        # visual = Marker()
        # visual.type = Marker.MESH_RESOURCE
        # #visual.type = Marker.CUBE
        # visual.mesh_resource = "package://p_grip_description/assets/gripper.stl"
        # visual.scale.x = 1
        # visual.scale.y = 1
        # visual.scale.z = 1
        # visual.color.r = 0.0
        # visual.color.g = 0.5
        # visual.color.b = 0.5
        # visual.color.a = 1.0

        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        # marker_control.markers.append(visual)
        self.__marker.controls.append(marker_control)

        #axis_control = InteractiveMarkerControl()
        #axis_control.name = "6dof"
        #axis_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        #self.__marker.controls.append(axis_control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.__marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.__marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.__marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.__marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.__marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.__marker.controls.append(control)

        server.insert(self.__marker,self.callback_ros)
        server.applyChanges()

    def get_pos(self):
        return self.__pos

    def get_orn(self):
        return self.__orn

    def callback_ros(self,feedback):
        self.__pos = np.array([feedback.pose.position.x,feedback.pose.position.y,feedback.pose.position.z])
        self.__orn = np.array([feedback.pose.orientation.w,feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z])

        feedback_msg = Pose()
        feedback_msg.position = feedback.pose.position
        feedback_msg.orientation = feedback.pose.orientation
        self._marker_pose_pub.publish(feedback_msg)

    def update_id(self,new_id,server):
        self.__uid = "marker_"+str(new_id)
        self.__marker.name = self.__uid
        server.insert(self.__marker,self.callback_ros)
        server.applyChanges()

    def get_name(self):
        return self.__uid

def main():

    rospy.init_node("ik_node")

    marker_server = InteractiveMarkerServer("keypoints_marker")
    tip_frame = rospy.get_param("/ik_node/tip_frame","pen_link")
    base_frame = rospy.get_param("/ik_node/base_frame","world")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    time.sleep(1);


    pose = tf_buffer.lookup_transform(base_frame,tip_frame,rospy.Time(0))

    pos = [pose.transform.translation.x,pose.transform.translation.y,pose.transform.translation.z]
    orn = [pose.transform.rotation.w,pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z]

    kp = Keypoint(marker_server,"marker_ik",base_frame,pos,orn)

    rospy.spin()

if __name__ == "__main__":
    main()
