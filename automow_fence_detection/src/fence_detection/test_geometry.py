#!/usr/bin/env python

import unittest

from geometry import Point

class TestPoint(unittest.TestCase):
	def test_point(self):
		point = Point(0,0) 

	def test_fromPolar(self):
		point = Point.from_polar(0.4370, -1.7487)
		print point
		

if __name__ == '__main__':
	unittest.main()