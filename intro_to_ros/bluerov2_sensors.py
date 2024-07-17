#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

import numpy as np

class problem1(Node):
    def __init__(self):
        super().__init__("problem1_messages")
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.battery = self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.callbackBattery,
            qos_profile
        )
        self.battery
        self.BatteryState = BatteryState()
        self.get_logger().info("starting battery sensor node")


        self.IMU = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.callbackImu,
            qos_profile
        )
        self.IMU 
        self.Imu = Imu()
        self.get_logger().info("starting IMU data node")

        self.battery_timer = self.create_timer(5.0, self.voltagecheck)

    def callbackBattery(self, msg):
        self.BatteryState = msg
        self.get_logger().info(f"\nBattery Voltage {self.BatteryState.current} {self.BatteryState.voltage}")

    def callbackImu(self, msg):
        self.Imu = msg
        #self.get_logger().info(f"\nImu data {self.Imu.header}")

    def voltagecheck(self):
        if self.BatteryState.voltage < 12:
            self.get_logger().info("battery voltage is too low")
        else:
            self.get_logger().info("battery voltage is safe")


def main(args=None):
    rclpy.init(args=args)
    node = problem1()
    print(node.BatteryState.header.stamp)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()