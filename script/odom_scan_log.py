#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This node generate text files from 'odom' and 'scan' topics
"""

import os
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from std_srvs.srv import SetBool

class OdomScanLog():
    """ Node class """

    def __init__(self):

        self.srv_name = 'rosbag_play/pause_playback'
        self.n_dig = 5
        self.format = '{:.' + str(self.n_dig) + 'f}' # '{:.5f}'.format(num)
        self.ini_time = 0

        # Filenames
        if rospy.has_param('out_dir'):
            out_dir = rospy.get_param('out_dir')
        else:
            out_dir = os.environ['HOME']
        odom_filename = out_dir + '/odom.txt'
        scan_filename = out_dir + '/scan.txt'

        # Waiting for available topics
        to_wait = ['odom', 'scan']
        while len(to_wait) > 0 and not rospy.is_shutdown():
            rospy.loginfo('Waiting: [' + " ".join(to_wait) + ']')
            current_topics = rospy.get_published_topics()
            for topic in current_topics:
                topic_name = topic[0][1:]
                for wait in to_wait:
                    if wait == topic_name:
                        to_wait.remove(wait)
        #rospy.sleep(1)

        # Node subscribers
        self.sub1 = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.sub2 = rospy.Subscriber('scan', LaserScan, self.scan_cb)

        # Open file for writing
        self.odom_fd = open(odom_filename, 'w')
        self.scan_fd = open(scan_filename, 'w')

        # Enable rosbag play (paused: false)
        rospy.loginfo('Starting rosbag playback...')
        rospy.wait_for_service(self.srv_name)
        try:
            set_bool = rospy.ServiceProxy(self.srv_name, SetBool)
            set_bool(False)
        except rospy.ServiceException as exc:
            rospy.logwarn('Service call failed: %s'%exc)

        rospy.loginfo('Writing files...')
        rospy.loginfo(' - ' + odom_filename)
        rospy.loginfo(' - ' + scan_filename)

    def odom_cb(self, msg):
        """ Odometry subscriber callback """
        if self.ini_time == 0:
            self.ini_time = msg.header.stamp

        quat = msg.pose.pose.orientation
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        #line = str(msg.header.stamp.secs) + '.'
        timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        line = self.format.format(timestamp) + '\t'
        line += self.format.format(msg.pose.pose.position.x) + '\t'
        line += self.format.format(msg.pose.pose.position.y) + '\t'
        line += self.format.format(yaw) + '\t'
        line += self.format.format(msg.twist.twist.linear.x) + '\t'
        line += self.format.format(msg.twist.twist.angular.z) + '\n'
        self.odom_fd.write(line)
        #rospy.loginfo('- Odom log line: ' + line)

    def scan_cb(self, msg):
        """ Scan subscriber callback """
        if self.ini_time == 0:
            self.ini_time = msg.header.stamp
        #line = str(msg.header.stamp.secs) + '.'
        timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        line = self.format.format(timestamp) + '\t'
        for ranges in msg.ranges:
            line += self.format.format(ranges) + '\t'
        line = line[:-1] + '\n'
        self.scan_fd.write(line)
        #rospy.loginfo('- Scan log line: ' + line)

if __name__ == '__main__':
    rospy.init_node('odom_scan_log')
    OdomScanLog()
    rospy.spin()
