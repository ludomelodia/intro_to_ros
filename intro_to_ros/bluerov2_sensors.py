#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
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


        self.diffpressure = self.create_subscription(
            FluidPressure,
            "/mavros/imu/diff_pressure",
            self.callbackDiffPressure,
            qos_profile
        )
        self.diffpressure
        self.FluidPressure = FluidPressure()
        self.get_logger().info("starting differential pressure node")


        self.staticpressure = self.create_subscription(
            FluidPressure,
            "/mavros/imu/static_pressure",
            self.callbackStaticPressure,
            qos_profile
        )
        self.staticpressure
        self.FluidPressure = FluidPressure()
        self.get_logger().info("starting static pressure node")

        self.battery_timer = self.create_timer(5.0, self.voltagecheck)
        """creates function to check voltage every 5 seconds"""

    def callbackBattery(self, msg):
        """Callback function for the battery. Prints battery voltage"""
        self.BatteryState = msg
        self.get_logger().info(f"\nBattery Voltage {self.BatteryState.voltage}")

    def callbackImu(self, msg):
        """callback function for the battery. Prints IMU header data"""
        self.Imu = msg
        self.get_logger().info(f"\nImu data {self.Imu.linear_acceleration}")

    def callbackDiffPressure(self,msg):
        """callback function for differential pressure"""
        self.Imu = msg
        self.get_logger().info(f"\nImu data {self.FluidPressure.fluid_pressure}")

    def callbackStaticPressure(self,msg):
        """callback function for static pressure"""
        self.Imu = msg
        self.get_logger().info(f"\nImu data {self.FluidPressure.fluid_pressure}")

    def voltagecheck(self):
        """checks if battery voltage is safe"""
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