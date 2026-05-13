#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import re, os, sys, struct
import numpy
import scipy.io

from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.stores.ros2_humble import (
    builtin_interfaces__msg__Time as Time,
    std_msgs__msg__Header as Header,
    sensor_msgs__msg__NavSatFix as NavSatFix,
    sensor_msgs__msg__NavSatStatus as NavSatStatus,
    sensor_msgs__msg__PointCloud2 as PointCloud2,
    sensor_msgs__msg__PointField as PointField,
    geographic_msgs__msg__GeoPointStamped as GeoPointStamped,
    geographic_msgs__msg__GeoPoint as GeoPoint,
)

def usec_to_rosbags_ns(time_us):
    return int(time_us * 1000)  # microseconds to nanoseconds

def gps2navsat(filename, writer, typestore, navsat_conn, geopoint_conn):
    with open(filename, 'rb') as f:
        try:
            while True:
                data = struct.unpack('qddd', f.read(8*4))
                time_us = data[0]
                lat_lon_el_theta = struct.unpack('dddd', f.read(8*4))
                gps_cov = struct.unpack('d'*16, f.read(8*16))

                if abs(lat_lon_el_theta[0]) < 1e-1:
                    continue

                stamp = Time(sec=int(time_us // 1_000_000),
                             nanosec=int((time_us % 1_000_000) * 1000))

                status = NavSatStatus(status=0, service=1)  # STATUS_FIX, SERVICE_GPS
                header = Header(stamp=stamp, frame_id='gps')
                cov = numpy.array(gps_cov).reshape(4,4)[:3,:3].flatten().astype(float)
                navsat = NavSatFix(
                    header=header, status=status,
                    latitude=lat_lon_el_theta[0],
                    longitude=lat_lon_el_theta[1],
                    altitude=lat_lon_el_theta[2],
                    position_covariance=cov,
                    position_covariance_type=2)  # COVARIANCE_TYPE_KNOWN
                
				 ts_ns = usec_to_rosbags_ns(time_us)
                writer.write(navsat_conn, ts_ns,
                             typestore.serialize_cdr(navsat, 'sensor_msgs/msg/NavSatFix'))

                gp = GeoPoint(latitude=lat_lon_el_theta[0],
                              longitude=lat_lon_el_theta[1],
                              altitude=lat_lon_el_theta[2])
                geopoint = GeoPointStamped(header=header, position=gp)
                writer.write(geopoint_conn, ts_ns,
                             typestore.serialize_cdr(geopoint, 'geographic_msgs/msg/GeoPointStamped'))
        except struct.error:
            print('done')

def main():
    if len(sys.argv) < 2:
        print('usage: ford2bag.py output_dirname')
        return
    output = sys.argv[1]

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with Writer(output) as writer:
        navsat_conn = writer.add_connection('/gps/fix',  'sensor_msgs/msg/NavSatFix',  typestore=typestore)
        gp_conn     = writer.add_connection('/gps/geopoint', 'geographic_msgs/msg/GeoPointStamped', typestore=typestore)
        pc_conn     = writer.add_connection('/velodyne_points', 'sensor_msgs/msg/PointCloud2', typestore=typestore)

        gps2navsat('GPS.log', writer, typestore, navsat_conn, gp_conn)

        filenames = sorted('SCANS/' + x for x in os.listdir('SCANS') if re.match(r'Scan[0-9]*\.mat', x))
        for fn in filenames:
            m = scipy.io.loadmat(fn)
            scan = numpy.transpose(m['SCAN']['XYZ'][0][0]).astype(numpy.float32)
            stamp_us = m['SCAN']['timestamp_laser'][0][0][0][0]
            # build PointCloud2 here using typestore.serialize_cdr
            # (I'm leaving the PointCloud2 build details out – the rosbags docs cover it)


if __name__ == '__main__':
    main()