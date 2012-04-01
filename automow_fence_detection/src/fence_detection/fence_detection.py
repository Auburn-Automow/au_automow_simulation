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

