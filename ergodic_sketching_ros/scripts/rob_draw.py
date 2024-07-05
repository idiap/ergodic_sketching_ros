# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import os
import pathlib
import argparse
from functools import partial
from datetime import datetime

import numpy
import cv2
import threading
import queue
import matplotlib.pyplot as plt

import rospy
import actionlib
import ergodic_sketching_msgs.msg
import ergodic_sketching_msgs.srv
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
from nav_msgs.msg import Path
import trajectory_msgs.msg

class Planner:
    def __init__(self, image: numpy.ndarray, log_dir: pathlib.Path, joint_state_pub) -> None:
        self._image = image

        self._stop_ctrl_thread_event = threading.Event()
        self._traj_queue = queue.Queue()
        self._joint_state_pub = joint_state_pub

        self._ctrl_thread = threading.Thread(target=self._ctrl_thread_cb)
        self._ctrl_thread.start()

        self._client = actionlib.SimpleActionClient("/ilqr_planner_ros/plan",ergodic_sketching_msgs.msg.PlannerAction)
        self._client.wait_for_server()
        self._log_dir = log_dir
        self._traj = []
        self._commands = []
        self._costs = []

    def start(self,path: Path)->None:
        goal = ergodic_sketching_msgs.msg.PlannerGoal()
        goal.path = path
        self._client.send_goal(goal,feedback_cb=self._planner_feedback)
        self._client.wait_for_result()
        self._stop_ctrl_thread_event.set()

        log_dir = self._log_dir / datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        log_dir.mkdir()

        costs = numpy.asarray(self._costs)
        joint_positions = numpy.asarray(self._traj)
        joint_velocities = numpy.asarray(self._commands)

        info = f"cost: min={costs.min()}, max={costs.max()}, std={costs.std()}, mean={costs.mean()} \n"
        info += f"q: min={joint_positions.min(axis=0)}, max={joint_positions.max(axis=0)}, std={joint_positions.std(axis=0)}, mean={joint_positions.mean(axis=0)} \n"
        info += f"dq: min={joint_velocities.min(axis=0)}, max={joint_velocities.max(axis=0)}, std={joint_velocities.std(axis=0)}, mean={joint_velocities.mean(axis=0)} \n"

        info_file = log_dir / "sumary.txt"
        info_file.touch()
        info_file.write_text(info)

        numpy.savetxt(str(log_dir/"costs.csv"),costs,delimiter=",")
        numpy.savetxt(str(log_dir/"joint_positions.csv"),joint_positions,delimiter=",")
        numpy.savetxt(str(log_dir/"joint_velocities.csv"),joint_velocities,delimiter=",")

        cv2.imwrite(str(log_dir/"image.png"),self._image)

        self._ctrl_thread.join()


    def _planner_feedback(self, feedback: trajectory_msgs.msg.JointTrajectory):
        traj = []
        cmd = []
        for i,p in enumerate(feedback.traj.points): # Go through the received trajectory
            traj += [p.positions]
            cmd += [p.velocities]

        self._traj_queue.put(traj) # add trajectory in the queue. Other thread will process it

        self._costs += [feedback.cost]
        self._traj += traj
        self._commands += cmd

    def _ctrl_thread_cb(self):

        if self._joint_state_pub is None:
            return

        rate = rospy.Rate(500)
        while True:
            if self._stop_ctrl_thread_event.is_set() and self._traj_queue.empty():
                return

            try:
                traj = self._traj_queue.get()
                for q in traj:
                    self._joint_state_pub(q)
                    rate.sleep()
            except queue.Empty:
                continue

def drozbot()->None:

    def _publish_joint_state(pub: rospy.Publisher,joint_names: list[str], q: numpy.ndarray) -> None:
        msg = JointState()
        msg.name = joint_names
        msg.position = q
        pub.publish(msg)

    parser = argparse.ArgumentParser()
    parser.add_argument("-i","--image",dest="image_path", type=str, help="image file to draw")
    parser.add_argument("-l","--logdir",dest="log_dir",type=str,help="log dir")
    parser.add_argument("--no-publish",dest="no_publish", action="store_true",help="Flag to not publish joint state further")
    args = parser.parse_args()

    log_path = pathlib.Path(args.log_dir)
    if not log_path.exists():
        log_path.mkdir()

    image_path = pathlib.Path(args.image_path)
    if not image_path.exists() and args.image_path != "random":
        raise FileNotFoundError(f"provided image file \"{str(image_path)}\" does not exist!")

    if not image_path.suffix.endswith((".jpg",".png",".jpeg")) and args.image_path != "random":
        raise Exception(f"provided image file \"{str(image_path)}\" is not in the expected format (jpg or png)!")

    os.system(f"rosparam dump {str(log_path)}/ros_config.yaml")
    rospy.init_node("drozbot")

    publish_joint_state = None
    if not args.no_publish:
        joint_states_pub = rospy.Publisher("rob_sim/joint_states",JointState,queue_size=10)
        joint_names = rospy.get_param("/ilqr_planner_ros/joint_names")
        publish_joint_state = partial(_publish_joint_state,joint_states_pub,joint_names)

    path_pub = rospy.Publisher("rob_sim/path",Path,queue_size=10)

    sketch_srv = rospy.ServiceProxy("/ergodic_sketching_ros/sketch",ergodic_sketching_msgs.srv.sketch)

    if args.image_path.lower() != "random": # Generate a random noise image, used to test.
        image = cv2.imread(str(image_path))
    else:
        image = numpy.random.rand(930,640,3) * 255
        image = image.astype(numpy.uint8)

    bridge = CvBridge()

    req = ergodic_sketching_msgs.srv.sketchRequest()
    req.drawing_zone_idx.data = 0
    req.image = bridge.cv2_to_imgmsg(image,"bgr8")
    resp = sketch_srv(req)
    path_pub.publish(resp.path)

    planner = Planner(image,log_path,publish_joint_state)
    planner.start(resp.path)

if __name__ == "__main__":
    drozbot()
