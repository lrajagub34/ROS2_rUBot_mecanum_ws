import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarTest(Node):

    def __init__(self):
        super().__init__('lidar_test_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.last_print_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.angle_offset_deg = 180.0  # lidar mounted backwards

    def wrap_angle(self, angle_deg):
        """Wrap to [-180, 180)."""
        while angle_deg >= 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    def angle_to_index(self, angle_deg, scan):
        angle_rad = math.radians(angle_deg)
        return int((angle_rad - scan.angle_min) / scan.angle_increment)

    def listener_callback(self, scan):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_print_time < 1:
            return

        # ----- Compute the 3 angles with offset -----
        angle_0     = self.wrap_angle(0   + self.angle_offset_deg)
        angle_neg90 = self.wrap_angle(-90 + self.angle_offset_deg)
        angle_pos90 = self.wrap_angle(90  + self.angle_offset_deg)

        # ----- Convert to indices -----
        index_0     = self.angle_to_index(angle_0, scan)
        index_neg90 = self.angle_to_index(angle_neg90, scan)
        index_pos90 = self.angle_to_index(angle_pos90, scan)

        # ----- All indices guaranteed valid now -----
        dist_0     = scan.ranges[index_0]
        dist_neg90 = scan.ranges[index_neg90]
        dist_pos90 = scan.ranges[index_pos90]

        self.get_logger().info("---- LIDAR readings ----")
        self.get_logger().info(f"Distance at 0°: {dist_0:.2f} m")
        self.get_logger().info(f"Distance at -90°: {dist_neg90:.2f} m")
        self.get_logger().info(f"Distance at +90°: {dist_pos90:.2f} m")

        # ----- Find closest -----
        valid = [(d, i) for i, d in enumerate(scan.ranges)
                 if d not in (0.0, float('inf'))]

        if valid:
            closest, idx = min(valid)
            angle_min_deg = math.degrees(scan.angle_min)
            angle_inc_deg = math.degrees(scan.angle_increment)

            angle_closest = angle_min_deg + idx * angle_inc_deg
            angle_closest -= self.angle_offset_deg
            angle_closest = self.wrap_angle(angle_closest)

            self.get_logger().info("---- Closest point ----")
            self.get_logger().info(f"Minimum distance: {closest:.2f} m at angle {angle_closest:.2f}°")
            
        self.last_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = LidarTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
