
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
	def __init__(self, msg=None):
		if msg:
			self.theta = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
			self.points = []
			self.angle_min = msg.angle_min
			self.angle_max = msg.angle_max
			self.increment = msg.angle_increment

			for (r, phi) in zip(msg.ranges, self.theta):
				self.points.append(Point.from_polar(r, phi))

	def plot(self, plt, color='blue'):
		x = [p.x for p in self.points]
		y = [p.y for p in self.points]
		plt.scatter(x,y, color=color)

	def update_theta(self):
		self.theta = np.arange(self.angle_min, self.angle_max, self.increment)

class Line(object):
	def __init__(self, point1, point2):
		pass