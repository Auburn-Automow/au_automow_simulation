import roslib; roslib.load_manifest('fence_detection')

import rospy

import fence_detection.geometry as geometry

import numpy as np

from matplotlib.pyplot import *
from matplotlib.lines import Line2D
import unittest

def laser_errorbar(scan, n_devs, plotrange=None):
    if plotrange is None:
        plotrange = [0, len(scan)]
    fig = figure()
    ax = fig.add_axes([0.1, 0.1, 5, 5], polar=True)
    ax.scatter(scan.ranges[0,plotrange[0]:plotrange[1]], scan.ranges[1,plotrange[0]:plotrange[1]], marker='+')
    ax.hold()
    for ii in np.arange(plotrange[0],plotrange[1],1):
        barlen = scan.ranges[2,ii] * n_devs
        newline = Line2D([scan.ranges[0,ii],scan.ranges[0,ii]], [scan.ranges[1,ii] - barlen, scan.ranges[1,ii] + barlen], lw=1)
        ax.add_line(newline)

if __name__ == '__main__':
    from fence_detection.utilities.bagfilehelper import BagFileHelper

    bagfile = '/Users/mjcarroll/devel/automow/au_automow_simulation/automow_fence_detection/data/fence-scan.bag'
    bag = BagFileHelper(bagfile)
    (topic, msg, timestamp) = bag.read(200, ['/scan'])

    scan = geometry.Scan.from_LaserScan(msg)

    laser_errorbar(scan, 4)
    show()
