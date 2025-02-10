#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from control_msgs.msg import JointJog
import json
import time
import math


class ListenNode(Node):
    def __init__(self):
        super().__init__("listener_twist_py")
        self.get_logger().info("Listener Python node has been created")
        self.subscriber_ = self.create_subscription(
            Twist, 
            '/Astro/cmd_vel', 
            self.subscriber_callback, 
            10)
        self.pubJog = self.create_publisher(
            JointJog, 
            '/Astro/wheels_jog', 
            10)
        self.radius = 0.108/2
        
        self.timerCounter = 0
        self.timer = self.create_timer(0.1, self.timerCB)



    def subscriber_callback(self, msg):
        self.timerCounter = 0
        print("PosX %.2f  AngZ %.2f"%(
            msg.linear.x,
            msg.angular.z
        ))
        jog = JointJog()
        jog.joint_names= ["LEFT", "RIGHT"]
        (left, right) = self.calcDifferential(msg.linear.x, msg.angular.z)
        jog.velocities=[left, right]
        print(jog)
        self.pubJog.publish(jog)

    def timerCB(self):
        self.timerCounter = self.timerCounter + 1
        if (self.timerCounter > 10):
            jog = JointJog()
            jog.joint_names= ["LEFT", "RIGHT"]
            jog.velocities=[0.0, 0.0]
            #print(jog)
            self.pubJog.publish(jog)
            self.timerCounter = 0

    def calcDifferential(self, vx, theta):
        if (theta == 0.0):
            rps = vx / self.radius
            return (rps, rps)
        L = 180 / 1000
        W = 340 / 1000
        Rb = L / math.tan(theta)
        vl = vx * ((Rb - W/2) / Rb)
        vr = vx * ((Rb + W/2) / Rb)

        lrps = vl / self.radius
        rrps = vr / self.radius
        return (lrps, rrps)

    
        


def main(args=None):
    rclpy.init(args=args)
    
    node = ListenNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()