#! /usr/bin/env python

import rospy
import rosunit
import unittest
import rostest
import time
PKG = 'pointcloud_merger'
NAME = 'pointcloud_merger_test'

class TestPCM(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node')

    def test_merge(self):
        self.assertTrue((True), "Integration error")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPCM)
