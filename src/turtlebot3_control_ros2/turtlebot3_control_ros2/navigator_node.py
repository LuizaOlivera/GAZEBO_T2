import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class GazeboBotController(Node):
    def __init__(self):
        super().__init__('gazebo_bot_controller')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.all_missions = [
            [  # Objetivo 1
                (-2.0, 2.0),
                (0.36, 1.84),
                (0.57, 0.91),
                (1.75, 0.98),
                (2.0, 2.0),
            ],
            [  # Objetivo 2
                (-2.0, 2.0),
                (0.42, 1.84),
                (1.37, -0.18),
                (2.10, -1.89),
            ],
            [  # Objetivo 3
                (-2.0, 2.0),
                (-0.55, 1.92),
                (-1.0, -0.09),
                (-1.0, -2.0), 
                (-2.16, -2.15), 
            ],
            [  # Objetivo 4
                (-2.0, 2.0),
                (-0.55, 1.92),
                (-0.55, -0.08),
                (-2.0, -0.08), 
                (-1.90, 1.20),
            ],             
        ]

        self.obj_index = 0
        self.is_returning = False
        self.load_path()

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw_angle = 0.0

        self.precision = 0.25
        self.vel_linear = 0.2
        self.vel_angular = 0.5

        self.timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info('[GAZEBO_T2] Navegação iniciada.')

    def load_path(self):
        self.route = self.all_missions[self.obj_index]
        self.return_route = list(reversed(self.route))
        self.route_step = 0
        self.get_logger().info(f"[GAZEBO_T2] Iniciando trajeto para alvo {self.obj_index + 1}.")

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        orient = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        self.yaw_angle = math.atan2(siny_cosp, cosy_cosp)

    def navigate(self):
        if self.route_step >= len(self.route):
            return

        goal = self.route[self.route_step]
        dx = goal[0] - self.pos_x
        dy = goal[1] - self.pos_y
        dist = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        diff = self.adjust_angle(angle - self.yaw_angle)

        move = Twist()

        if dist > self.precision:
            if abs(diff) > 0.2:
                move.angular.z = self.vel_angular * (diff / abs(diff))
                move.linear.x = 0.0
            else:
                move.linear.x = min(self.vel_linear, 0.5 * dist)
                move.angular.z = 0.0
        else:
            self.get_logger().info(f"[GAZEBO_T2] Checkpoint {self.route_step} concluído: {goal}")
            self.route_step += 1

            if self.route_step == len(self.route):
                if not self.is_returning:
                    self.get_logger().info("[GAZEBO_T2] Objeto alcançado. Voltando à base.")
                    time.sleep(2)
                    self.is_returning = True
                    self.route = self.return_route
                    self.route_step = 1
                else:
                    self.get_logger().info("[GAZEBO_T2] Retornou à origem.")

                    self.obj_index += 1
                    if self.obj_index < len(self.all_missions):
                        self.is_returning = False
                        self.load_path()
                    else:
                        self.get_logger().info("[GAZEBO_T2] Todas as missões foram realizadas.")
                        self.publisher.publish(Twist())
                        rclpy.shutdown()
                        return

        self.publisher.publish(move)

    def adjust_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = GazeboBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

