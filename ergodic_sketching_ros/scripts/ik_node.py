# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import pathlib

import rospy

import numpy
from PyLQR.sim import KDLRobot
from PyLQR.utils import Sd

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class IkSolver:

    def __init__(self, urdf_path: pathlib.Path, base_frame: str, tip_frame: str, q_max: numpy.ndarray, q_min: numpy.ndarray, dof: int) -> None:

        self._robot = KDLRobot(urdf_path,base_frame,tip_frame,[0]*dof,[0]*dof,[0]*3,[0]*3,False)
        self._dof = dof
        self._q_max = q_max
        self._q_min = q_min

    def set_q_start(self,q_start: numpy.ndarray):
        self._q_start = q_start

    def ik(self, pos: numpy.ndarray, orn: numpy.ndarray, nb_iter:int=400):

        q = self._q_start
        for _ in range(nb_iter):
            self._robot.set_conf(q,[0]*self._dof,False)
            cur_pos = self._robot.get_ee_pos()
            cur_orn = self._robot.get_ee_orn()

            pos_error = pos - cur_pos
            orn_error = -2 * Sd.dquat_to_w_jac(orn) @ Sd.logMap(orn,cur_orn)

            error = numpy.hstack((pos_error,orn_error))
            jac_inv = numpy.linalg.pinv(self._robot.J())

            q = q + 0.001 * jac_inv @ error

            for i,q_i in enumerate(q):
                if q_i > self._q_max[i]:
                    q[i] = self._q_max[i]

                if q_i < self._q_min[i]:
                    q[i] = self._q_min[i]

            if numpy.linalg.norm(error) < 1e-4:
                break

        return q

class IKManager:

    def __init__(self) -> None:

        self._js_sub = rospy.Subscriber("~joint_states",JointState,self._joint_states_cb)
        self._pose_sub = rospy.Subscriber("~target_pose",Pose,self._pose_cb)
        self._js_pub = rospy.Publisher("~new_joint_states",JointState,queue_size=10)

        self._base_frame = rospy.get_param("~base_frame")
        self._tip_frame = rospy.get_param("~tip_frame")
        self._dof = rospy.get_param("~dof")
        self._urdf_path = rospy.get_param("/robot_description")
        self._joint_names = rospy.get_param("~joint_names")
        self._q_max = rospy.get_param("~q_max")
        self._q_min = rospy.get_param("~q_min")

        rospy.loginfo(f"base_frame: {self._base_frame}")
        rospy.loginfo(f"tip_frame: {self._tip_frame}")
        rospy.loginfo(f"dof: {self._dof}")
        rospy.loginfo(f"urdf_path: {self._urdf_path}")
        rospy.loginfo(f"joint_names: {self._joint_names}")
        rospy.loginfo(f"q_max: {self._q_max}")
        rospy.loginfo(f"q_min: {self._q_min}")

        self._ik_solver = IkSolver(self._urdf_path,self._base_frame,self._tip_frame,self._q_max,self._q_min,self._dof)

    def _joint_states_cb(self,msg: JointState):
        q = []
        for joint_name in self._joint_names:
            q+= [msg.position[msg.name.index(joint_name)]]

        self._ik_solver.set_q_start(q)

    def _pose_cb(self, msg: Pose):

        pos = [msg.position.x,msg.position.y,msg.position.z]
        orn = [msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z]

        q = self._ik_solver.ik(pos,orn)
        resp = JointState()
        resp.name = self._joint_names
        resp.position = q
        resp.velocity = [0] * self._dof
        resp.effort = [0] * self._dof
        self._js_pub.publish(resp)

    def start(self):
        rospy.spin()


if __name__ == "__main__":

    rospy.init_node("inverse_kinematics")
    IKManager().start()
