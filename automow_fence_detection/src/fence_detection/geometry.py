
import math
import numpy as np

class Point(object):
	def __init__(self, x, y):
		self._x = x
		self._y = y

	@property
	def x(self):
		return self._x

	@property
	def y(self):
		return self._y

	@classmethod
	def from_polar(cls, r, phi):
		x = r * math.cos(phi)
		y = r * math.sin(phi)
		return Point(x,y)

	@classmethod
	def distance(cls, point1, point2):
		return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

	def __str__(self):
		return str(self._x) + " " + str(self._y)

class Scan(object):
	def __init__(self):
		self.angle_min = 0
		self.angle_max = 0
		self.angle_increment = 0
		self.ranges = 0
		pass

	@classmethod
	def from_LaserScan(cls, scan_msg):
		ret = Scan()

		ret.angle_min = scan_msg.angle_min
		ret.angle_max = scan_msg.angle_max
		ret.angle_increment = scan_msg.angle_increment

		temp = np.zeros((4,len(scan_msg.ranges)), dtype=np.float32)
		theta = np.arange(ret.angle_min, ret.angle_max, ret.angle_increment)
		try:
			temp[0,:] = theta
		except:
			temp[0,:] = np.arange(ret.angle_min, ret.angle_max + ret.angle_increment, ret.angle_increment)
		temp[1,:] = scan_msg.ranges
		temp[2,:] = temp[1,:] * np.cos(temp[0,:])
		temp[3,:] = temp[1,:] * np.sin(temp[0,:])
		ret.ranges = temp
		return ret

	def __len__(self):
		return self.ranges.shape[1]


class Line(object):
	def __init__(self, point1, point2, ):
		pass

	@property
	def alpha(self):
		return self._alph