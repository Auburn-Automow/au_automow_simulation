#!/usr/bin/env python

import roslib; roslib.load_manifest('automow_fence_detection')

import rospy
from rospy.numpy_msg import numpy_msg
import rosbag
from sensor_msgs.msg import LaserScan

import numpy as np

from geometry import Point, Scan

import matplotlib.pyplot as plt

global scan

def read_bag(increment):
	bag = rosbag.Bag('/Users/mjcarroll/devel/automow/au_automow_simulation/automow_fence_detection/data/fence-scan.bag')
	
	bag_gen = bag.read_messages(topics=['/scan'])
	for i in range(increment):
		bag_gen.next()

	(topic, msg, time) = bag_gen.next()
	bag.close()
	return (topic, msg, time)

def segment_scan(scan):
	clusters = []
	for i in range(len(scan)):
			


def main():
	global scan
	(topic, msg, time) = read_bag(400)
	scan = Scan.from_LaserScan(msg)


if __name__ == '__main__':
	main()
