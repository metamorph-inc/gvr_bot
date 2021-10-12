#!/usr/bin/env python

import math as m
import json
import os.path as path

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import LinkStates
from rospy_message_converter import message_converter

#MIN_THRESHOLD = 2.0  # meters
#STUCK_THRESHOLD = 0.2  # meters
#MAX_STUCK_TIME = 6  # seconds


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
    def __init__(self, ros_rate, goal_xy_positions=None):

        self.rate = ros_rate
        self.goal_xy_positions = goal_xy_positions

        self.odometry = []
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
            self._read_tb_manifest()

        # Simple Action Client
        move_base_action = '/move_base'
        self.move_base_action_client = SimpleActionClient(move_base_action, MoveBaseAction)
        rospy.loginfo("  Waiting for MoveBaseActionServer at %s ...", move_base_action)
        self.move_base_action_client.wait_for_server()  # block until action server starts
        rospy.loginfo("  MoveBaseActionServer AVAILABLE.")

        # Subscribers
        self.current_xy_position = None
        self.prev_xy_position = None
        self.stuck_time = None
        self.odometry_subscriber = rospy.Subscriber('odom', Odometry, self._odometry_cb)

        self.gazebo_links_subscriber = rospy.Subscriber('/gazebo/link_states', LinkStates, self._link_states_cb)
        self._sphere_link_name = None
        self._sphere_link_index = None
        self.odometry_publisher = rospy.Publisher('odom', Odometry)
        self._prev_odom_record_time = None

    def execute(self):
        rospy.loginfo("Waiting for simulation to start + 1.0 sec")
        while not rospy.is_shutdown() and rospy.get_time() < 1.0:
            self.rate.sleep()

        waypoint_index = -1
        while not rospy.is_shutdown():
            waypoint_index += 1
            if waypoint_index < len(self.goal_xy_positions):
                self.send_goal_to_movebase_action_server(*self.goal_xy_positions[waypoint_index])
                while self.move_base_action_client.get_state() != GoalStatus.SUCCEEDED:
                    self.rate.sleep()

            else:
                break  # no more waypoints!

        # record total simulation time
        self._metrics['simulation_time'] = rospy.get_time()
        # Save odometry data to file
        testbench_folder_path = path.dirname(self.openmeta_testbench_manifest_path)
        with open(path.join(testbench_folder_path, "odometry.json"), 'w') as savefile:
            json_str = json.dumps(self.odometry, sort_keys=True, indent=2, separators=(',', ': '))
            savefile.write(json_str)
        self._fileoutputs['odometry_file'] = "odometry.json"

        if self.openmeta_testbench_manifest_path is not None:
            self._update_tb_manifest()
            self._write_tb_manifest()

    def send_goal_to_movebase_action_server(self, x, y):
        rospy.loginfo("Send MoveBaseAction request")
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1
        self.move_base_action_client.send_goal(goal)

    def _odometry_cb(self, msg):
        # TODO: Replace with ROS message filtering
        if self._prev_odom_record_time is None:
            self._prev_odom_record_time = rospy.get_time()
        elif rospy.get_time() - self._prev_odom_record_time > 0.1:
            odom_dict = message_converter.convert_ros_message_to_dictionary(msg)
            self.odometry.append(odom_dict)
            self._prev_odom_record_time = rospy.get_time()

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

    def _link_states_cb(self, msg):
        """

        :param msg:
        :type msg: LinkStates
        :return:
        """
        if self._sphere_link_name is None:
            for i, name in enumerate(msg.name):
                if "sphere" in name.lower():
                    self._sphere_link_name = name
                    self._sphere_link_index = i
                    break
        else:
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "sphere"
            odom.pose.pose = msg.pose[self._sphere_link_index]
            odom.pose.covariance = [0]*36
            odom.twist.twist = msg.twist[self._sphere_link_index]
            odom.twist.covariance = [0]*36
            self.odometry_publisher.publish(odom)

    def _read_tb_manifest(self):
        # Parameters
        self._params = {}
        for tb_param in self._testbench_manifest['Parameters']:
            self._params[tb_param['Name']] = tb_param['Value']  # WARNING: If you use these values - make sure to check the type

        # Metrics
        self._metrics = {}
        for tb_metric in self._testbench_manifest['Metrics']:
            self._metrics[tb_metric['Name']] = tb_metric['Value']

        # File Inputs
        self._fileinputs = {}
        for tb_fileinput in self._testbench_manifest['FileInputs']:
            self._fileinputs[tb_fileinput['Name']] = tb_fileinput['FileName']

        # File Outputs
        self._fileoutputs = {}
        for tb_fileoutput in self._testbench_manifest['FileOutputs']:
            self._fileoutputs[tb_fileoutput['Name']] = tb_fileoutput['FileName']

    def _update_tb_manifest(self):
        # Metrics
        for tb_metric in self._testbench_manifest['Metrics']:
            if tb_metric['Name'] in self._metrics:
                tb_metric['Value'] = self._metrics[tb_metric['Name']]

        # File Outputs
        for tb_fileoutput in self._testbench_manifest['FileOutputs']:
            if tb_fileoutput['Name'] in self._fileoutputs:
                tb_fileoutput['Value'] = self._fileoutputs[tb_fileoutput['Name']]

    def _write_tb_manifest(self):
        # Save updated testbench_manifest.json
        with open(self.openmeta_testbench_manifest_path, 'w') as savefile:
            json_str = json.dumps(self._testbench_manifest, sort_keys=True, indent=2, separators=(',', ': '))
            savefile.write(json_str)


if __name__ == '__main__':
    rospy.init_node('gvr_bot_waypoints_simulation')
    rate = rospy.Rate(100)

    xy_waypoints = [(0, 8), (8, 8), (-8, 0), (0, 0)]
    testbench = GVRBotSimulationTestbench(ros_rate=rate, goal_xy_positions=xy_waypoints)
    testbench.execute()
