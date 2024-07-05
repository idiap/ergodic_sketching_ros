# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import pathlib
import numpy
import argparse
from functools import partial

import rospy
from sensor_msgs.msg import JointState

def replay():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t","--topic",dest="topic", default="rob_sim/joint_states", type=str, help="topic to publish joint_states")
    parser.add_argument("-f","--folder",dest="folder",type=str,help="log_folder")
    parser.add_argument("-r","--rate",dest="rate",default=100,type=int,help="publish rate")
    parser.add_argument("-s","--start",dest="start_percentage",default=0,type=float,help="where to start (in percent)")
    parser.add_argument("-n","--joint-names-topic",dest="joint_names_topic",default="/ilqr_planner_ros/joint_names", type=str,help="topic where to get joint names")
    args = parser.parse_args()

    rospy.init_node("replay_node")

    replay_folder = pathlib.Path(args.folder)
    rate = args.rate

    if not replay_folder.exists():
        print(f"Replay folder ({str(replay_folder)}) does not exists")
        return

    joint_positions_csv = replay_folder / "joint_positions.csv"
    if not joint_positions_csv.exists():
        print(f"Replay folder ({str(replay_folder)}) does not contain a joint_positions.csv file!")
        return

    joint_positions = numpy.loadtxt(str(joint_positions_csv),delimiter=",")

    print("will compute velocities by finite differences (rate will be used as dt)")
    joint_velocities = numpy.array([numpy.zeros(joint_positions.shape[1])]+[ (joint_positions[i] - joint_positions[i-1]) * rate for i in range(1,joint_positions.shape[0]) ])

    joint_names = rospy.get_param(args.joint_names_topic,default=None)
    if joint_names is None:
        print(f"Joint names topic ({args.joint_names_topic}) is not published!")
        return

    def _publish_joint_state(pub: rospy.Publisher,joint_names: list[str], q: numpy.ndarray, dq: numpy.ndarray) -> None:
        msg = JointState()
        msg.name = joint_names
        msg.position = q
        msg.velocity = dq
        pub.publish(msg)

    joint_states_pub = rospy.Publisher(args.topic,JointState,queue_size=10)
    publish_joint_state = partial(_publish_joint_state,joint_states_pub,joint_names)

    start_idx = int(joint_positions.shape[0]*args.start_percentage)
    rate = rospy.Rate(rate)
    for q,dq in zip(joint_positions[start_idx:],joint_velocities[start_idx:]):
        publish_joint_state(q,dq)
        rate.sleep()

if __name__=="__main__":
    replay()
