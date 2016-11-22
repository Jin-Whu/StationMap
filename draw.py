#!/usr/bin/env python
# coding:utf-8

import sys
import os
import pyproj
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
from enum import Enum, unique


@unique
class Projection(Enum):
    ecef = 1
    geo = 2

    @staticmethod
    def members():
        return Projection.__members__.keys()

    @staticmethod
    def get(key):
        return Projection.__members__.get(key)


class Station(object):
    def __init__(self):
        self.path = None
        self.projection = None
        self.range = list()
        self.lat = list()
        self.lon = list()
        self.station = list()
        self._readconfig()

    def _readconfig(self):
        configpath = os.path.join(os.path.dirname(__file__), 'configure.ini')
        if not os.path.exists(configpath):
            print 'Not find configure.ini'
            sys.exit()
        with open(configpath) as f:
            for line in f:
                if line.startswith('projection'):
                    projection = line.split('=')[1].strip()
                    if projection not in Projection.members():
                        print 'Not support %s' % projection
                        sys.exit()
                    self.projection = Projection.get(projection)
                if line.startswith('lat'):
                    pieces = line.split('=')[1].split()
                    if len(pieces) == 3:
                        try:
                            maxlat = float(pieces[0])
                            minlat = float(pieces[1])
                            step = float(pieces[2])
                            self.range.extend([maxlat, minlat, step])
                        except:
                            pass
                if line.startswith('lon'):
                    pieces = line.split('=')[1].split()
                    if len(pieces) == 3:
                        try:
                            maxlon = float(pieces[0])
                            minlon = float(pieces[1])
                            step = float(pieces[2])
                            self.range.extend([maxlon, minlon, step])
                        except:
                            pass

                if line.startswith('path'):
                    path = line.split('=')[1].strip()
                    if not os.path.exists(path):
                        print 'Not find valid path: %s' % path
                        sys.exit()
                    self.path = path

    def read(self):
        """Read station coordiante."""
        if not os.path.exists(self.path):
            print 'Not find %s' % self.path
            sys.exit()
        if self.projection == Projection.ecef:
            length = 4
            self._readecef(length)
        elif self.projection == Projection.geo:
            length = 3
            self._readgeo(length)

    def _readgeo(self, length):
        with open(self.path) as f:
            for line in f:
                pieces = line.split()
                if len(pieces) != length:
                    continue
                try:
                    station = pieces[0]
                    lon = pieces[1]
                    lat = pieces[2]
                    self.station.append(station)
                    self.lon.append(lon)
                    self.lat.append(lat)
                except:
                    continue

    def _readecef(self, length):
        ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
        lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
        with open(self.path) as f:
            for line in f:
                pieces = line.split()
                if len(pieces) != length:
                    continue
                try:
                    station = pieces[0]
                    x = float(pieces[1])
                    y = float(pieces[2])
                    z = float(pieces[3])

                    # convert xyz to blh
                    lon, lat, alt = pyproj.transform(
                        ecef, lla, x, y, z, radians=False)
                    self.station.append(station)
                    self.lon.append(lon)
                    self.lat.append(lat)
                except:
                    continue

    def draw(self):
        plt.style.use('ggplot')

        # determin range of map
        if not self.range:
            maxlat = max(self.lat)
            minlat = min(self.lat)
            maxlon = max(self.lon)
            minlon = min(self.lon)
            steplat = (maxlat - minlat) / 4
            steplon = (maxlon - minlon) / 4
        else:
            minlat = self.range[0]
            maxlat = self.range[1]
            steplat = self.range[2]
            minlon = self.range[3]
            maxlon = self.range[4]
            steplon = self.range[5]

        ax = plt.gca()
        m = Basemap(
            projection='cyl',
            urcrnrlat=maxlat,
            llcrnrlat=minlat,
            urcrnrlon=maxlon,
            llcrnrlon=minlon,
            ax=ax)

        m.drawparallels(
            np.arange(minlat, maxlat + steplat, steplat),
            labels=[1, 1, 0, 0],
            linewidth=0,
            size=10,
            weight='bold')
        m.drawmeridians(
            np.arange(minlon, maxlon + steplon, steplon),
            labels=[0, 0, 1, 1],
            linewidth=0,
            size=10,
            weight='bold')

        m.drawcountries(color='#778899')
        m.drawlsmask(land_color='#F0E68C', ocean_color='#87CEEB', lakes=False)

        m.scatter(self.lon, self.lat, latlon=True, marker='o', s=50)

        for station, lat, lon in zip(self.station, self.lat, self.lon):
            x, y = m(lon, lat)
            ax.text(x, y, station, size=5, weight='bold')
        figpath = os.path.join(os.path.split(self.path)[0], 'station.png')
        plt.savefig(figpath, bbox_inches='tight')
        plt.clf()
        plt.close()


if __name__ == '__main__':
    station_d = Station()
    station_d.read()
    station_d.draw()
