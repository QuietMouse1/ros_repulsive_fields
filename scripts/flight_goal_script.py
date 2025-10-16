#!/usr/bin/env python3

# Simulate repulsive potential fields 
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import time

finished = False
goal_search_radius = 0.75

# +x = forward, -y = right, +z = up
class FlightGoal:
    def __init__(self):
        rospy.init_node("flight_goal")
        self.odom_sub = rospy.Subscriber("/livox_Odometry", Odometry, self.odom_callback)
        self.nav_goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.current_goal_idx = 0
        self.publish_first = False
        # rospy.sleep(1.0)

        # goal poses
        self.goal_poses = []
        tmp_goal_pose = PoseStamped()
        tmp_goal_pose.pose.position.x = 5.0
        tmp_goal_pose.pose.position.y = 0.0
        tmp_goal_pose.pose.position.z = 1.5
        tmp_goal_pose.pose.orientation.w = 1.0
        tmp_goal_pose.header.frame_id = "world"

        tmp_goal_pose1 = PoseStamped()
        tmp_goal_pose1.pose.position.x = 5.0
        tmp_goal_pose1.pose.position.y = -5.0
        tmp_goal_pose1.pose.position.z = 1.5
        tmp_goal_pose1.pose.orientation.w = 1.0
        tmp_goal_pose1.header.frame_id = "world"

        tmp_goal_pose2 = PoseStamped()
        tmp_goal_pose2.pose.position.x = 0.0
        tmp_goal_pose2.pose.position.y = -5.0
        tmp_goal_pose2.pose.position.z = 1.5
        tmp_goal_pose2.pose.orientation.w = 1.0
        tmp_goal_pose2.header.frame_id = "world"

        tmp_goal_pose3 = PoseStamped()
        tmp_goal_pose3.pose.position.x = 0.0
        tmp_goal_pose3.pose.position.y = 0.0
        tmp_goal_pose3.pose.position.z = 1.5
        tmp_goal_pose3.pose.orientation.w = 1.0
        tmp_goal_pose3.header.frame_id = "world"

        self.goal_poses.append(tmp_goal_pose)
        self.goal_poses.append(tmp_goal_pose1)
        self.goal_poses.append(tmp_goal_pose2)
        self.goal_poses.append(tmp_goal_pose3)

        # self.nav_goal_publisher.publish(self.goal_poses[self.current_goal_idx])
        rospy.spin()

    def odom_callback(self, msg):
        global finished
        if not self.publish_first:
            self.publish_first = True
            self.nav_goal_publisher.publish(self.goal_poses[self.current_goal_idx])
            # self.nav_goal_publisher.publish(self.goal_poses[self.current_goal_idx])
            # self.nav_goal_publisher.publish(self.goal_poses[self.current_goal_idx])
            print("FIRST PUBLISH!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        if not finished:
            destination_goal = self.goal_poses[self.current_goal_idx]

            v1 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            v2 = np.array([destination_goal.pose.position.x, destination_goal.pose.position.y])

            # Compute magnitudes
            mag1 = np.linalg.norm(v1)
            mag2 = np.linalg.norm(v2)

            # Magnitude difference
            diff = abs(mag1 - mag2)
            print("Absolute distanace to goal {}".format(diff))
            # self.nav_goal_publisher.publish(self.goal_poses[self.current_goal_idx])
            if (diff < goal_search_radius):
                print("Reached destination {}!".format(self.current_goal_idx))
                self.current_goal_idx = self.current_goal_idx + 1
                next_goal_x = self.goal_poses[self.current_goal_idx.pose.position.x]
                next_goal_y = self.goal_poses[self.current_goal_idx.pose.position.y]
                next_goal_z = self.goal_poses[self.current_goal_idx.pose.position.z]

                print("Publishing next goal at x:{}, y:{}, z:{}!".format(next_goal_x, next_goal_y, next_goal_z))
                self.nav_goal_publisher.publish(self.goal_poses[self.current_goal_idx])
                if (self.current_goal_idx + 1 == len(destination_goal)):
                    print("EVERY GOAL IS REACHED!!")
                    finished = True
            else:
                pass


if __name__ == "__main__":
    FlightGoal()