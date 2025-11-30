import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.5)    # desired distance to right wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # band around base_distance (RIGHT)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        # Periodic cmd_vel publisher at 10 Hz (0.1 s)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower (RIGHT tol, BACK_RIGHT when closest) - differential drive."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
        self._shutting_down = True

        # Set last command to zero
        self.cmd = Twist()

        # Try a final publish (publisher may still be valid even if shutdown started)
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # Context/publisher may already be invalid -> ignore
            pass

        # Cancel timers safely
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # If the context or publisher is invalid, ignore
            pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # 1. DEFINIM TOTES LES ZONES (Ara incloem l'esquerra)
        FRONT       = []
        FR_RIGHT    = []
        RIGHT       = []
        BACK_RIGHT  = []
        FR_LEFT     = []  # Nova zona

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif 20 < ang <= 70:    # Afegim visió Front-Esquerra
                FR_LEFT.append(d)

        # Minimal distances
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_fr_left    = min(FR_LEFT)    if FR_LEFT    else float('inf')

        twist = Twist()
        action = ""

        #----------------------------------------------------------
        # RULE 1: FRONT obstacle → turn left (Emergència frontal)
        #----------------------------------------------------------
        if min_front < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.v_ang * 2.0
            action = f"FRONT {min_front:.2f} m → turn LEFT"

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT obstacle → slow + turn left
        #----------------------------------------------------------
        elif min_fr_right < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.v_ang * 2.0
            action = f"FRONT-RIGHT {min_fr_right:.2f} m → turn LEFT"

        #----------------------------------------------------------
        # NOVE RULE: FRONT-LEFT obstacle → turn right 
        # (Si xoquem per l'esquerra, girem cap a la paret dreta)
        #----------------------------------------------------------
        elif min_fr_left < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = -self.v_ang * 2.0
            action = f"FRONT-LEFT {min_fr_left:.2f} m → turn RIGHT"

        #----------------------------------------------------------
        # RULE 3: RIGHT visible → control with tolerance band
        #----------------------------------------------------------
        elif math.isfinite(min_right):
            error = min_right - self.base_distance

            if abs(error) <= self.tol:
                # Inside band: go straight
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = f"RIGHT OK ({min_right:.2f}m) → STRAIGHT"

            elif error < 0:
                # Too close (més a prop del target) → Girem Esquerra
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = self.v_ang 
                action = f"RIGHT CLOSE ({min_right:.2f}m) → Turn LEFT"

            else:
                # Too far (més lluny del target) → Girem Dreta
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = -self.v_ang * 2
                action = f"RIGHT FAR ({min_right:.2f}m) → Turn RIGHT"

        #----------------------------------------------------------
        # RULE 4: BACK-RIGHT → Si perdem la paret lateral, però la veiem al darrere
        #----------------------------------------------------------
        elif math.isfinite(min_back_right):
            twist.linear.x = self.v_lin * 0.5
            twist.linear.y = 0.0
            twist.angular.z = -self.v_ang * 1.5
            action = f"BACK-RIGHT {min_back_right:.2f} m → Pivot RIGHT"

        #----------------------------------------------------------
        # NOVA RULE: LOST → Si no veiem res, busquem la paret
        #----------------------------------------------------------
        else:
            # Avancem i girem molt suaument a la dreta per trobar paret
            twist.linear.x = self.v_lin 
            twist.linear.y = 0.0
            twist.angular.z = -self.v_ang * 0.3
            action = "LOST (No wall) → Search (Arc Right)"

        # Update last commanded twist
        self.cmd = twist

        # Logging
        if action != self._last_action_logged:
            self.get_logger().info(action if action else "Stopped.")
            self._last_action_logged = action

        self._state_action = action if action else "Stopped"

    #--------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
