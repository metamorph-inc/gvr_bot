#!/usr/bin/env python

import math as m
import json

import rospy
from actionlib import SimpleActionClient

from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

MIN_THRESHOLD = 2.0  # meters
STUCK_THRESHOLD = 0.2#0.05  # meters
MAX_STUCK_TIME = 6#0  # seconds


def load_json_file(filename):
    print("Opening {} ...".format(filename))
    json_dict = None
    try:
        with open(filename) as f_in:
            json_dict = json.load(f_in)
    except IOError:
        print("No {} file found.".format(filename))
        pass
    return json_dict


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

        self.times = []
        self._initial_x_position = None
        self._initial_y_position = None
        self._initial_z_position = None
        self.x_abs_positions = []
        self.y_abs_positions = []
        self.z_abs_positions = []
        self.openmeta_testbench_manifest_path = rospy.get_param('~openmeta/testbench_manifest_path', None)
        if self.openmeta_testbench_manifest_path is not None:
            self._testbench_manifest = load_json_file(self.openmeta_testbench_manifest_path)

            # TestBench Parameters
            self._params = {}
            for tb_param in self._testbench_manifest['Parameters']:
                self._params[tb_param['Name']] = tb_param['Value']  # WARNING: If you use these values - make sure to check the type

            # TestBench Metrics
            self._metrics = {}
            for tb_metric in self._testbench_manifest['Metrics']:  # FIXME: Hmm, this is starting to look a lot like OpenMDAO...
                self._metrics[tb_metric['Name']] = tb_metric['Value']

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

        if self.openmeta_testbench_manifest_path is not None:
            self._metrics['time'] = self.times
            self._metrics['absolute_position_x'] = self.x_abs_positions
            self._metrics['absolute_position_y'] = self.y_abs_positions
            self._metrics['absolute_position_z'] = self.z_abs_positions
            self._metrics['displacement_x'] = [x - self._initial_x_position for x in self.x_abs_positions]
            self._metrics['displacement_y'] = [y - self._initial_y_position for y in self.y_abs_positions]
            self._metrics['displacement_z'] = [z - self._initial_z_position for z in self.z_abs_positions]
            self._metrics['final_absolute_position_x'] = self.x_abs_positions[-1]
            self._metrics['final_absolute_position_y'] = self.y_abs_positions[-1]
            self._metrics['final_absolute_position_z'] = self.z_abs_positions[-1]
            self._metrics['final_displacement_x'] = self._metrics['displacement_x'][-1]
            self._metrics['final_displacement_y'] = self._metrics['displacement_y'][-1]
            self._metrics['final_displacement_z'] = self._metrics['displacement_z'][-1]
            self._write_metrics_to_tb_manifest()

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
        self.times.append(msg.header.stamp.secs)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.current_xy_position = [x, y]
        self.x_abs_positions.append(x)
        self.y_abs_positions.append(y)
        self.z_abs_positions.append(z)
        if self.prev_xy_position is None:
            self.prev_xy_position = self.current_xy_position
            self.stuck_time = rospy.get_time()
            self._initial_x_position = x
            self._initial_y_position = y
            self._initial_z_position = z

    def _write_metrics_to_tb_manifest(self):
        # Write to testbench_manifest metric
        for tb_metric in self._testbench_manifest['Metrics']:
            if tb_metric['Name'] in self._metrics:
                tb_metric['Value'] = self._metrics[tb_metric['Name']]

        # Save updated testbench_manifest.json
        with open(self.openmeta_testbench_manifest_path, 'w') as savefile:
            json_str = json.dumps(self._testbench_manifest, sort_keys=True, indent=2, separators=(',', ': '))
            savefile.write(json_str)


if __name__ == '__main__':
    testbench = GVRBotSimulationTestbench()

