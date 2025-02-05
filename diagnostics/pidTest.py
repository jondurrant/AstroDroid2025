#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import PidState
from std_msgs.msg import Float32
from std_msgs.msg import String
import json
import time


class TestPid(Node):
    def __init__(self, vels):
        super().__init__("listener_node_py")
        self.get_logger().info("Listener Python node has been created")
        self.subscriber_ = self.create_subscription(
            PidState, 
            '/Astro/Wheel3E2C/pid', 
            self.subscriber_callback, 
            10)
        self.testName = "None"
        self.testStart = False
        self.testVels = vels
        self.testCount = 0

        self.timer_ = self.create_timer(10, self.timerCB)

        self.pubVel = self.create_publisher(
            Float32, 
            '/Astro/Wheel3E2C/velocity', 
            10)
        
        self.pubPIDConfig = self.create_publisher(
            String, 
            '/Astro/Wheel3E2C/set_config', 
            10)
        


    def subscriber_callback(self, msg):
        data = msg
        #self.get_logger().info(f'received data: {data}')
        if (self.testStart):
            print("%s, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f"%(
                self.testName,
                self.testTarget,
                data.output,
                data.error,
                data.p_error,
                data.i_error,
                data.d_error,
                data.p_term,
                data.i_term,
                data.d_term
            ))

    def startTest(self, name, vel):
        self.testName = name
        self.testStart = True
        self.testTarget = vel
        self.pubVelocity(vel)

    def stopTest(self):
        self.testStart = False
        self.pubVelocity(0.0)

    def pubVelocity(self, vel):
        msg = Float32()
        msg.data = vel
        self.pubVel.publish(msg)
    
    def pubConfig(self, kP, kI, kD):
        data = {
            "PID_KP": kP,
            "PID_KI": kI,
            "PID_KD": kD,
            "REBOOT": True
        }
        msg = String()
        msg.data = json.dumps(data)

        self.pubPIDConfig.publish(msg)

    def timerCB(self):
        self.stopTest()
        time.sleep(2)
        if (self.testCount >= len(self.testVels)):
            return
        v = self.testVels[self.testCount]
        test = "Test %.1f"%v
        self.startTest(test, v)
        self.testCount = self.testCount + 1
        


def main(args=None):
    print("Test Name, Target, Current, Error, P Err, I Err, D Err, KP, KI, KD")
    rclpy.init(args=args)
    vels = [0.5, 1.0, 2.0, 3.0]
    node = TestPid(vels)
    node.pubConfig(0.2, 0.04, 0.1)
    time.sleep(5)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()