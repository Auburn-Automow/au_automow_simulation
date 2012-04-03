import roslib; roslib.load_manifest('automow_fence_detection')

import rospy
import rosbag
from sensor_msgs.msg import LaserScan

import numpy as np
import unittest

from fence_detection.geometry import Scan
from fence_detection.utilities.bagfilehelper import BagFileHelper

bagfile = '/Users/mjcarroll/devel/automow/au_automow_simulation/automow_fence_detection/data/fence-scan.bag'


bag = BagFileHelper(bagfile)
(topic, msg, timestamp) = bag.read(200, ['/scan'])

class Test_Scan(unittest.TestCase):
	def setUp(self):
		self.scan = Scan.from_LaserScan(msg)

	def test_fromMsg(self):
		self.assertAlmostEqual(-2.356, self.scan.angle_min, places=3)
		self.assertAlmostEqual(2.356, self.scan.angle_max, places=3)
		self.assertAlmostEqual(0.0061, self.scan.angle_increment, places=3)

	def test_partitiona_bad_start(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_angle(-3, 0)
		self.assertEqual(cm.exception.message, "Start angle must be >= angle_min")

	def test_paritiona_bad_end(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_angle(0, 3)
		self.assertEqual(cm.exception.message, "End angle must be <= angle_max")

	def test_paritiona_bad_start_boundary(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_angle(self.scan.angle_min - 0.001, 0)
		self.assertEqual(cm.exception.message, "Start angle must be >= angle_min")

	def test_partitiona_bad_end_boundary(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_angle(0, self.scan.angle_max + 0.001)
		self.assertEqual(cm.exception.message, "End angle must be <= angle_max")

	def test_paritiona_bad_pair(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_angle(0,0)
		self.assertEqual(cm.exception.message, "Start must be < end")

	def test_partition_bad_start(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_index(-1, 10)
		self.assertEqual(cm.exception.message, "Start index must be >= 0")

	def test_partition_bad_end(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_index(10, 1000)
		self.assertEqual(cm.exception.message, "End index must be <= len(scan)")

	def test_partition_bad_end_boundary(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_index(10, len(self.scan)+1)

	def test_partition_bad_pair(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_index(100, 10)
		self.assertEqual(cm.exception.message, "Start must be < end")

	def test_partition_bad_pair_equal(self):
		with self.assertRaises(ValueError) as cm:
			self.scan.get_partition_by_index(10, 10)
		self.assertEqual(cm.exception.message, "Start must be < end")

	def test_partition_length(self):
		newscan = self.scan.get_partition_by_index(0,len(self.scan)-1)
		self.assertEqual(len(newscan), len(self.scan))

	def test_parition_angles(self):
		newscan = self.scan.get_partition_by_index(0, 9)
		self.assertEqual(len(newscan), 10)

	def test_partition_maxmin(self):
		newscan = self.scan.get_partition_by_index(100,102)
		self.assertAlmostEqual(newscan.range_min, 1.187, places=3)
		self.assertAlmostEqual(newscan.range_max, 1.237, places=3)

	def test_paritiona(self):
		newscan = self.scan.get_partition_by_angle(self.scan.angle_min, self.scan.angle_max)
		self.assertEqual(len(newscan), len(self.scan))



if __name__ == '__main__':
	unittest.main()