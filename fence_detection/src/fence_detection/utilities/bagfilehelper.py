import roslib; roslib.load_manifest('fence_detection')

import rospy
import rosbag

class BagFileHelper(object):
	def __init__(self, bagfile):
		self.bagfile = bagfile

	def read(self, increment, topics=['/scan']):
		bag = rosbag.Bag(self.bagfile)
		bag_gen = bag.read_messages(topics=topics)
		for i in range(increment):
			bag_gen.next()
		(topic, msg, timestamp) = bag_gen.next()
		bag.close()
		return (topic, msg, timestamp)
