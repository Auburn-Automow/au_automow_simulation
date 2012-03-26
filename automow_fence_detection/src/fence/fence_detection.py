#!/usr/bin/env python

import roslib; roslib.load_manifest('automow_fence_detection')

import rospy
from rospy.numpy_msgs import numpy_msgs
import rosbag
from sensor_msgs.msg import LaserScan

import numpy as np

from geometry import Point, Scan

import matplotlib.pyplot as plt

def read_bag(increment):
	bag = rosbag.Bag('/Users/mjcarroll/devel/automow/au_automow_simulation/automow_fence_detection/data/fence-scan.bag')
	
	bag_gen = bag.read_messages(topics=['/scan'])
	for i in range(increment):
		bag_gen.next()

	(topic, msg, time) = bag_gen.next()
	bag.close()
	return (topic, msg, time)


def cluster_data(msg, threshold):

	theta = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

	scanClusters = []
	segmentBegin = 0
	for i in range(len(msg.ranges)-1):
		if abs(msg.ranges[i] - msg.ranges[i+1]) > threshold:
			cluster = Scan()
			cluster.angle_min = theta[segmentBegin]
			cluster.angle_max = theta[i]
			cluster.increment = msg.angle_increment

			cluster.theta = theta[i:segmentBegin]

			for (r,phi) in zip(msg.ranges[i:segmentBegin], cluster.theta):
				cluster.points.append(Point.from_polar(r,phi))

			scanClusters.append(cluster)
			segmentBegin = i+1
	return scanClusters

def plot_segments(plt, clusters):
	for cluster in clusters:
		plt.plot((0, 5*np.cos(cluster.angle_min)),
				 (0, 5*np.sin(cluster.angle_max)))

if __name__ == '__main__':
	(topic, msg, time) = read_bag(400)
	scan = Scan(msg)
	scanClusters = cluster_data(msg, 1.0)
	
	plt.axis('equal')
	scan.plot(plt)
	plot_segments(plt, scanClusters)
	
	plt.show()

	
