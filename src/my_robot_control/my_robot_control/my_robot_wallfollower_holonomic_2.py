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
        self.declare_parameter('turn_speed', 0.50)       # angular speed (Utilitzat per reset)
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
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower Holonomic + ROTATION RESET on Left Trap."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        if self._shutting_down: return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()
        try: self.publisher.publish(self.cmd)
        except Exception: pass
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try: t.cancel()
            except Exception: pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        if self._shutting_down: return
        try: self.publisher.publish(self.cmd)
        except Exception: pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action with ROTATION RESET if trapped on left."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # 1. Definició de Zones
        FRONT      = []
        FR_RIGHT   = []
        RIGHT      = []
        BACK_RIGHT = []
        BACK       = []
        FR_LEFT    = [] # <--- Important per detectar xoc imminent
        LEFT       = [] # <--- Important per al gir

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d): continue
            if d < scan.range_min or d > scan.range_max: continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif ang < -160 or ang > 160:
                BACK.append(d)
            # ZONES ESQUERRA
            elif 20 < ang <= 70:
                FR_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)

        # Mínims
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')
        min_fr_left    = min(FR_LEFT)    if FR_LEFT    else float('inf')
        min_left       = min(LEFT)       if LEFT       else float('inf')

        twist = Twist()
        # Per defecte no girem (comportament holonòmic pur)
        twist.angular.z = 0.0 
        
        action = ""
        WALL_THRESHOLD = 1.2

        #----------------------------------------------------------
        # RULE DE RESET (Prioritat Alta): OBSTACLE A L'ESQUERRA
        # Si tenim paret a l'esquerra, hem caigut en un parany o estem al revés.
        # SOLUCIÓ: GIRAR (No fer strafe) per reorientar el robot.
        #----------------------------------------------------------
        if min_left < self.base_distance or min_fr_left < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            # Girem a l'esquerra per portar la paret del costat esquerre 
            # cap al davant i eventualment a la dreta.
            twist.angular.z = self.v_ang 
            action = f"LEFT DETECTED ({min(min_left, min_fr_left):.2f}) -> ROTATING LEFT (Reset)"

        #----------------------------------------------------------
        # RULE 1: FRONT -> Move Left (Slide)
        #----------------------------------------------------------
        elif min_front < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = self.v_lin  
            action = f"FRONT ({min_front:.2f}) -> SLIDE LEFT"

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT -> Move Diag Left
        #----------------------------------------------------------
        elif min_fr_right < self.base_distance:
            twist.linear.x = self.v_lin 
            twist.linear.y = self.v_lin
            action = f"FRONT-RIGHT ({min_fr_right:.2f}) -> DIAG FRONT-LEFT"

        #----------------------------------------------------------
        # RULE 3: RIGHT -> Maintain Parallel
        #----------------------------------------------------------
        elif math.isfinite(min_right) and min_right < WALL_THRESHOLD:
            error = min_right - self.base_distance

            if abs(error) <= self.tol:
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                action = "RIGHT OK -> FORWARD"

            elif error < 0:
                # Too close -> Forward + Slide Left (Away)
                twist.linear.x = self.v_lin
                twist.linear.y = self.v_lin * 0.6 
                action = "RIGHT CLOSE -> FWD + LEFT"
            else:
                # Too far -> Forward + Slide Right (In)
                twist.linear.x = self.v_lin
                twist.linear.y = -self.v_lin * 0.5
                action = "RIGHT FAR -> FWD + RIGHT"

        #----------------------------------------------------------
        # RULE 4: BACK-RIGHT -> Diag Front-Right
        #----------------------------------------------------------
        elif math.isfinite(min_back_right) and min_back_right < WALL_THRESHOLD:
            twist.linear.x = self.v_lin
            twist.linear.y = -self.v_lin 
            action = f"BACK-RIGHT ({min_back_right:.2f}) -> DIAG FRONT-RIGHT"

        #----------------------------------------------------------
        # RULE 5: BACK -> Slide Right
        #----------------------------------------------------------
        elif min_back < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = -self.v_lin 
            action = f"BACK ({min_back:.2f}) -> SLIDE RIGHT"

        #----------------------------------------------------------
        # LOST
        #----------------------------------------------------------
        else:
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            action = "LOST -> FORWARD"

        # Publish
        self.cmd = twist

        # Log
        if action != self._last_action_logged:
            self.get_logger().info(action if action else "No action.")
            self._last_action_logged = action
        self._state_action = action

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