#!/usr/bin/env python3

# Simulate repulsive potential fields 
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def create_cube_pointcloud(origin, size, resolution):
    xs = np.arange(origin[0], origin[0] + size[0] + resolution/2, resolution)
    ys = np.arange(origin[1], origin[1] + size[1] + resolution/2, resolution)
    zs = np.arange(origin[2], origin[2] + size[2] + resolution/2, resolution)

    points = [[x, y, z] for x in xs for y in ys for z in zs]

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    header = rospy.Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()

    pointcloud = pc2.create_cloud(header, fields, points)
    return pointcloud


t = 0.0
dt = 0.1  # time increment per loop
r = 1.0   # radius of circular motion
resolution = 0.25 # distance between points in the cube
cube_size = 1.0

if __name__ == "__main__":
    rospy.init_node("simulated_pointcloud")

    point_cloud_publisher = rospy.Publisher("/simulated_pointcloud", PointCloud2, queue_size=1)
    odom_publisher = rospy.Publisher("/odometry", Odometry, queue_size=1)
    rate = rospy.Rate(10)

    odom = Odometry()
    odom.header.frame_id = "map"
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0 
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = 0.0
    odom.pose.pose.orientation.y = 0.0
    odom.pose.pose.orientation.z = 0.0
    odom.pose.pose.orientation.w = 1.0  

    while not rospy.is_shutdown():
        x = r * np.sin(t)
        y = r * np.cos(t)
        z = 0.0
        t = t + dt
        # x = 1.0
        # y = 0.0

        size = np.array([cube_size, cube_size, cube_size])
        origin = np.array([x - cube_size/2, y - cube_size/2, z])
        pointcloud = create_cube_pointcloud(origin, size, resolution)
        print("Publishing simulated pointcloud")
        point_cloud_publisher.publish(pointcloud)
        odom_publisher.publish(odom)
        rate.sleep()
