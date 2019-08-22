#!/usr/bin/env python

import math as m

import rospy
from actionlib import SimpleActionClient

from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

MIN_THRESHOLD = 2.0  # meters
STUCK_THRESHOLD = 0.05  # meters
MAX_STUCK_TIME = 60  # seconds


def distance(pt1, pt2):
    if len(pt1) != len(pt2):
        raise ValueError("Received two points/vectors of unequal length! Both points/vectors must be of equal length")
    else:
        sum = 0
        for val1, val2 in zip(pt1, pt2):
            sum += (val1-val2)**2
        dist = m.sqrt(sum)
        return dist


class GVRBotSimulationTestbench(object):
    def __init__(self):
        rospy.init_node('gvr_bot_simulation_testbench')
        self.rate = rospy.Rate(100)

        # Simple Action Client
        self.goal_xy_position = [14, 6]  # FIXME: hardcoded
        move_base_action = '/move_base'
        self.move_base_action_client = SimpleActionClient(move_base_action, MoveBaseAction)
        rospy.loginfo("  Waiting for MoveBaseActionServer at %s ...", move_base_action)
        self.move_base_action_client.wait_for_server()  # block until action server starts
        rospy.loginfo("  MoveBaseActionServer AVAILABLE.")

        # Subscribers
        self.current_xy_position = None
        self.prev_xy_position = None
        self.stuck_time = None
        self.odometry_subscriber = rospy.Subscriber('/odometry/base_link', Odometry,
                                                    self._odometry_cb)

        rospy.loginfo("Waiting for simulation to start + 1.0 sec")
        while not rospy.is_shutdown() and rospy.get_time() < 1.0:
            self.rate.sleep()

        self.send_goal_to_movebase_action_server()

        while not rospy.is_shutdown():
            if self.current_xy_position is not None \
                    and distance(self.current_xy_position, self.goal_xy_position) < MIN_THRESHOLD:
                break  # if robot reaches goal position
            elif self.prev_xy_position is not None:
                if distance(self.current_xy_position, self.prev_xy_position) > STUCK_THRESHOLD:
                    self.prev_xy_position = self.current_xy_position
                    self.stuck_time = rospy.get_time()
                elif rospy.get_time() - self.stuck_time > MAX_STUCK_TIME:
                    break  # if robot has been stuck in one place for too long
            self.rate.sleep()

    def send_goal_to_movebase_action_server(self):
        rospy.loginfo("Send MoveBaseAction request")
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = self.goal_xy_position[0]
        goal.target_pose.pose.position.y = self.goal_xy_position[1]
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1
        self.move_base_action_client.send_goal(goal)

    def _odometry_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_xy_position = [x, y]
        if self.prev_xy_position is None:
            self.prev_xy_position = self.current_xy_position
            self.stuck_time = rospy.get_time()


if __name__ == '__main__':
    testbench = GVRBotSimulationTestbench()
