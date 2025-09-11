#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanComparer(Node):
    def __init__(self):
        super().__init__('scan_comparer')
        
        self.original_scan = None
        self.filtered_scan = None
        
        self.sub_original = self.create_subscription(
            LaserScan, '/scan', self.original_callback, 10)
        self.sub_filtered = self.create_subscription(
            LaserScan, '/scan_filtered', self.filtered_callback, 10)
            
        self.timer = self.create_timer(3.0, self.compare_scans)
        
    def original_callback(self, msg):
        self.original_scan = msg
        
    def filtered_callback(self, msg):
        self.filtered_scan = msg
        
    def compare_scans(self):
        if self.original_scan is None or self.filtered_scan is None:
            self.get_logger().info("Waiting for both scans...")
            return
            
        # Count infinite values
        orig_inf = sum(1 for x in self.original_scan.ranges if math.isinf(x))
        filt_inf = sum(1 for x in self.filtered_scan.ranges if math.isinf(x))
        
        # Count valid points in center (±15 degrees = ±0.2618 rad)
        angles = []
        total_points = len(self.original_scan.ranges)
        for i in range(total_points):
            angle = self.original_scan.angle_min + i * self.original_scan.angle_increment
            angles.append(angle)
            
        # Count valid points in the center region (±15 degrees)
        center_range = 0.2618  # 15 degrees in radians
        orig_center_valid = 0
        filt_center_valid = 0
        
        for i, angle in enumerate(angles):
            if -center_range <= angle <= center_range:
                if not math.isinf(self.original_scan.ranges[i]):
                    orig_center_valid += 1
                if not math.isinf(self.filtered_scan.ranges[i]):
                    filt_center_valid += 1
        
        self.get_logger().info("=== SCAN COMPARISON ===")
        self.get_logger().info(f"Original scan: {orig_inf} infinite values, {len(self.original_scan.ranges) - orig_inf} valid points")
        self.get_logger().info(f"Filtered scan: {filt_inf} infinite values, {len(self.filtered_scan.ranges) - filt_inf} valid points")
        self.get_logger().info(f"Valid points in center ±15°: Original={orig_center_valid}, Filtered={filt_center_valid}")
        self.get_logger().info(f"Difference: {filt_inf - orig_inf} more infinite values in filtered scan")
        
        if filt_inf > orig_inf:
            self.get_logger().info("✅ Filter is working! More points filtered out.")
        else:
            self.get_logger().warn("❌ Filter might not be working properly.")

def main():
    rclpy.init()
    node = ScanComparer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()