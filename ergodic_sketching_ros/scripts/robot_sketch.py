# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import os
import pathlib
import argparse
from datetime import datetime

import numpy
import cv2
import threading
import queue

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ergodic_sketching_msgs.srv import Sketch
from ergodic_sketching_msgs.action import Planner
import rclpy.publisher
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from rcl_interfaces.srv import GetParameters
from cv_bridge import CvBridge

class Sketcher(Node):
    def __init__(self, image: numpy.ndarray, log_dir: pathlib.Path, publish_js: bool):
        super().__init__("drozbot_client")

        self._image = image

        self._stop_ctrl_thread_event = threading.Event()
        self._traj_queue = queue.Queue()

        self._param_client = self.create_client(GetParameters, "/planner_action_server/get_parameters")
        self._param_client.wait_for_service()
        param_req = GetParameters.Request()
        param_req.names=["joint_names"]
        param_future = self._param_client.call_async(param_req)
        rclpy.spin_until_future_complete(self,param_future)
        param_res = param_future.result()
        self._joint_names = param_res.values[0].string_array_value

        self._joint_state_pub = None
        if publish_js:
            self._joint_state_pub = self.create_publisher(JointState,"/rob_sim/joint_states",10)

        self._path_pub= self.create_publisher(Path,"/rob_sim/path",10)
        
        self._sketch_client = self.create_client(Sketch,"/sketch")
        self._sketch_client.wait_for_service()

        self._ctrl_thread = threading.Thread(target=self._ctrl_thread_cb)        
        self._client = ActionClient(self,Planner,"/plan")

        self._log_dir = log_dir
        self._traj = []
        self._commands = []
        self._costs = []

    def publish_joint_state(self,q: numpy.ndarray):
        msg = JointState()
        msg.name = self._joint_names
        msg.position = q
        self._joint_state_pub.publish(msg)

    def process(self):
        bridge = CvBridge()
        sketch_req = Sketch.Request()
        sketch_req.drawing_zone_idx.data = 0
        sketch_req.image = bridge.cv2_to_imgmsg(self._image,"bgr8")
        sketch_future = self._sketch_client.call_async(sketch_req)
        rclpy.spin_until_future_complete(self,sketch_future)
        result = sketch_future.result()
        self._path_pub.publish(result.path)
        self._start_planner(result.path)

    def _start_planner(self,path: Path)->None:
        self._ctrl_thread.start()
        goal = Planner.Goal()
        goal.path = path
        self._client.wait_for_server()
        future = self._client.send_goal_async(goal,feedback_callback=self._planner_feedback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
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
        self.get_logger().info("Sketching over")

    def _planner_feedback(self, feedback):
        traj = []
        cmd = []
        for i,p in enumerate(feedback.feedback.traj.points): # Go through the received trajectory
            traj += [p.positions]
            cmd += [p.velocities]

        self._traj_queue.put(traj) # add trajectory in the queue. Other thread will process it

        self._costs += [feedback.feedback.cost]
        self._traj += traj
        self._commands += cmd

    def _ctrl_thread_cb(self):

        if self.publish_joint_state is None:
            return

        rate = self.create_rate(500)
        while True:
            if self._stop_ctrl_thread_event.is_set() and self._traj_queue.empty():
                return

            try:
                traj = self._traj_queue.get()
                for q in traj:
                    self.publish_joint_state(q)
                    rate.sleep()
            except queue.Empty:
                continue

def drozbot() -> None:
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

    if args.image_path.lower() != "random": # Generate a random noise image, used to test.
        image = cv2.imread(str(image_path))
    else:
        image = numpy.random.rand(930,640,3) * 255
        image = image.astype(numpy.uint8)
    
    rclpy.init()
    node = Sketcher(image,log_path,not args.no_publish)
    node.process()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    drozbot()