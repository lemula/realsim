#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Read nav_msgs/Odometry and extract the state needed by the real-car PID loop.

Default input is FAST-LIO's /Odometry topic. The node parses:
    x, y        from msg.pose.pose.position
    yaw         from msg.pose.pose.orientation
    speed       from msg.twist.twist.linear

It republishes a compact Float64MultiArray:
    [timestamp, x, y, speed, yaw, yaw_rate]

and can optionally write the same fields to CSV for offline replay with the
trajectory tracking code.
"""

import csv
import math
import os
from dataclasses import dataclass

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from common import wrap_to_pi


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return wrap_to_pi(math.atan2(siny_cosp, cosy_cosp))


@dataclass
class OdomState:
    timestamp: float
    x: float
    y: float
    speed: float
    yaw: float
    yaw_rate: float


class OdomStateReader:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.state_topic = rospy.get_param("~state_topic", "/realcar/state")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.speed_mode = rospy.get_param("~speed_mode", "norm")
        self.csv_path = rospy.get_param("~csv_path", "")
        self.debug_every = int(rospy.get_param("~debug_every", 20))

        self.csv_file = None
        self.csv_writer = None
        self.seq = 0

        if self.csv_path:
            parent = os.path.dirname(self.csv_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.DictWriter(
                self.csv_file,
                fieldnames=["timestamp", "x", "y", "speed", "yaw", "yaw_rate"],
            )
            self.csv_writer.writeheader()

        self.pub = rospy.Publisher(self.state_topic, Float64MultiArray, queue_size=20)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=100)

        rospy.on_shutdown(self.close)
        rospy.loginfo(
            "odom_state_reader start: odom_topic=%s state_topic=%s speed_mode=%s csv_path=%s",
            self.odom_topic,
            self.state_topic,
            self.speed_mode,
            self.csv_path or "(disabled)",
        )

    def parse_speed(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        if self.speed_mode == "x":
            return float(vx)
        if self.speed_mode == "xy_norm":
            return float(math.hypot(vx, vy))
        if self.speed_mode == "norm":
            return float(math.sqrt(vx * vx + vy * vy + vz * vz))
        rospy.logwarn_throttle(2.0, "unknown speed_mode=%s, fallback to norm", self.speed_mode)
        return float(math.sqrt(vx * vx + vy * vy + vz * vz))

    def parse_odom(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.Time.now().to_sec()

        pose = msg.pose.pose
        twist = msg.twist.twist
        return OdomState(
            timestamp=stamp,
            x=float(pose.position.x),
            y=float(pose.position.y),
            speed=self.parse_speed(msg),
            yaw=quaternion_to_yaw(pose.orientation),
            yaw_rate=float(twist.angular.z),
        )

    def publish_state(self, state):
        out = Float64MultiArray()
        out.layout.dim.append(MultiArrayDimension(label="state", size=6, stride=6))
        out.data = [state.timestamp, state.x, state.y, state.speed, state.yaw, state.yaw_rate]
        self.pub.publish(out)

    def write_csv(self, state):
        if self.csv_writer is None:
            return
        self.csv_writer.writerow(
            {
                "timestamp": round(state.timestamp, 9),
                "x": state.x,
                "y": state.y,
                "speed": state.speed,
                "yaw": state.yaw,
                "yaw_rate": state.yaw_rate,
            }
        )
        self.csv_file.flush()

    def odom_cb(self, msg):
        state = self.parse_odom(msg)
        self.seq += 1
        self.publish_state(state)
        self.write_csv(state)

        if self.debug_every > 0 and self.seq % self.debug_every == 0:
            rospy.loginfo(
                "[ODOM_STATE] seq=%d t=%.6f frame=%s x=%.4f y=%.4f speed=%.4f yaw=%.3fdeg yaw_rate=%.4f",
                self.seq,
                state.timestamp,
                msg.header.frame_id or self.frame_id,
                state.x,
                state.y,
                state.speed,
                math.degrees(state.yaw),
                state.yaw_rate,
            )

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None


if __name__ == "__main__":
    rospy.init_node("odom_state_reader")
    OdomStateReader()
    rospy.spin()
