import utm
import numpy as np

class statisticsRoute(object):

    def __init__(self):
        self.timeOfFly = 0
        self.distanceOfFly = 0

    @staticmethod
    def DistGPS(gps0, gps1, alt0=0, alt1=0):
        e0, n0, _, _ = utm.from_latlon(*gps0)
        e1, n1, _, _ = utm.from_latlon(*gps1)
        return np.linalg.norm([e0 - e1, n0 - n1, alt0 - alt1])


    def calculateStatistics(self, GPSCoords):

        for actual_p, next_p in zip(GPSCoords, GPSCoords[1:]):
            self.distanceOfFly += self.DistGPS(actual_p[0:2], next_p[0:2], 0, 0)