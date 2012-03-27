#!/usr/bin/env python

import roslib; roslib.load_manifest('automow_fence_detection')

import rospy
import rosbag
from sensor_msgs.msg import LaserScan

import numpy as np
import matplotlib.pyplot as plt

from matplotlib.projections import PolarAxes, register_projection
from matplotlib.transforms import Affine2D, Bbox, IdentityTransform

import time

bagfile = '/home/mjcarroll/devel/automow/au_automow_simulation/automow_fence_detection/data/fence-scan.bag'

class Scan(object):
	def __init__(self):
		self.angle_min = 0
		self.angle_max = 0
		self.angle_increment = 0
		self.ranges = 0

	@classmethod
	def from_LaserScan(cls, scan_msg):
		ret = Scan()

		ret.angle_min = scan_msg.angle_min
		ret.angle_max = scan_msg.angle_max
		ret.angle_increment = scan_msg.angle_increment

		temp = np.zeros((5,len(scan_msg.ranges)), dtype=np.float32)
		theta = np.arange(ret.angle_min, ret.angle_max, ret.angle_increment)
		try:
			temp[0,:] = theta
		except:
			temp[0,:] = np.arange(ret.angle_min, ret.angle_max + ret.angle_increment, ret.angle_increment)
		temp[1,:] = scan_msg.ranges
		for ii in range(len(temp[1,:])):
			if temp[1,ii] <= 1.0:
				temp[2,ii] = 0.03**2
			else:
				temp[2,ii] = (0.03 * temp[1,ii])**2
		temp[3,:] = temp[1,:] * np.cos(temp[0,:])
		temp[4,:] = temp[1,:] * np.sin(temp[0,:])

		ret.ranges = temp
		return ret

	def __len__(self):
		return self.ranges.shape[1]

	def polar_withvar(self, fig):
		ax = fig.add_axes([0.1, 0.1, 5, 5], polar=True)
		ax.scatter(self.ranges[0,:], self.ranges[1,:])


def segment_scan(scan, threshold, min_cluster):
    clusters = []
    clusterBegin = 0
    for ii in range(len(scan)-1):
        r1 = scan.ranges[1,ii]
        r2 = scan.ranges[1,ii+1]
        if (abs(r1-r2) >= threshold or ii == len(scan)-2 ):
            if (ii - clusterBegin) > min_cluster:
                cluster = Scan()
                cluster.angle_min = scan.ranges[0,clusterBegin]
                cluster.angle_max = scan.ranges[0,ii]
                cluster.angle_increment = scan.angle_increment
                cluster.ranges = scan.ranges[:,clusterBegin:ii]
                clusters.append(cluster)
                clusterBegin = ii + 1
    return clusters

def read_bag(bagfile, increment, topics=['/scan']):
	bag = rosbag.Bag(bagfile)
	bag_gen = bag.read_messages(topics=topics)
	for i in range(increment):
		bag_gen.next()
	(topic, msg, time) = bag_gen.next()
	bag.close()
	return (topic, msg, time)

def main():
	global scan
	(topic, msg, timestamp) = read_bag(bagfile, 400)
	scan = Scan.from_LaserScan(msg)

	fig = plt.figure(figsize=(4,4))
	ax = fig.add_axes([0.1, 0.1, 5, 5])

	fig.show()

if __name__ == '__main__':
	main()

