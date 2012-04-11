import numpy as np


class Scan(object):
    def __init__(self):
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.range_max = 0
        self.range_min = 0
        self.ranges = 0

    def __len__(self):
        return self.ranges.shape[1]

    def get_partition_by_index(self, start, end):
        """
        Get a new Scan object of [start:end] (inclusive) indicies
        """
        if start < 0:
            raise ValueError("Start index must be >= 0")
        if end > self.ranges.shape[1]:
            raise ValueError("End index must be <= len(scan)")
        if start >= end:
            raise ValueError("Start must be < end")

        ret = Scan()
        ret.angle_min = self.ranges[0, start]
        ret.angle_max = self.ranges[0, end]
        ret.angle_increment = self.angle_increment
        ret.timestamp = self.timestamp
        ret.ranges = self.ranges[:, start:end + 1].copy()

        ret.range_min = np.amin(ret.ranges, 1)[1]
        ret.range_max = np.amax(ret.ranges, 1)[1]
        return ret

    def get_partition_by_angle(self, start, end):
        """
        Get a new Scan object of [start:end] angles
        """
        if start < self.angle_min:
            raise ValueError("Start angle must be >= angle_min")
        if end > self.angle_max:
            raise ValueError("End angle must be <= angle_max")
        if start >= end:
            raise ValueError("Start must be < end")

        index_min = np.searchsorted(self.ranges[0, :], start, side='right') - 1
        index_max = np.searchsorted(self.ranges[0, :], end)

        return self.get_partition_by_index(index_min, index_max)

    @classmethod
    def from_LaserScan(cls, scan_msg):
        ret = Scan()

        ret.angle_min = scan_msg.angle_min
        ret.angle_max = scan_msg.angle_max
        ret.angle_increment = scan_msg.angle_increment
        ret.range_min = scan_msg.range_min
        ret.range_max = scan_msg.range_max
        ret.timestamp = scan_msg.header.stamp

        temp = np.zeros((5, len(scan_msg.ranges)), dtype=np.float32)
        theta = np.arange(ret.angle_min, ret.angle_max, ret.angle_increment)
        try:
            temp[0, :] = theta
        except:
            temp[0, :] = np.arange(ret.angle_min, (ret.angle_max +
                ret.angle_increment), ret.angle_increment)
        temp[1, :] = scan_msg.ranges
        for ii in range(len(temp[1, :])):
            if temp[1, ii] <= 1.0:
                temp[2, ii] = 0.03 ** 2
            else:
                temp[2, ii] = (0.03 * temp[1, ii]) ** 2
        temp[3, :] = temp[1, :] * np.cos(temp[0, :])
        temp[4, :] = temp[1, :] * np.sin(temp[0, :])

        ret.ranges = temp
        return ret
