import rclpy 
from std_msgs.msg import Bool
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import math
from tf2_geometry_msgs import do_transform_point


class PathFollowerCrta(Node):
    def __init__(self):
        super().__init__('crta_follower')

        self.path_sub = self.create_subscription(Path, '/astar_path', self.path_callback, 10)
        self.cmd_pub_drone = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_tracker_sub = self.create_subscription(Bool, '/stop_tracker', self.stop_tracker_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = []
        self.current_target_index = 0
        self.goal_reached = False 

        self.Kp = 1.5      
        self.max_vel = 3.0  
        self.timer = self.create_timer(0.1, self.timer_callback)  

        self.current_yaw = 0.0 


    def path_callback(self, msg):
        self.path = [pose.pose.position for pose in msg.poses]  
        self.current_target_index = 0
        self.goal_reached = False
        self.get_logger().info(f"Primljena je nova putanja sa: ({len(self.path)} točaka).")


    def quaternion_to_yaw(self, qx, qy, qz, qw):
        yaw_1 = 2.0 * (qw * qz + qx * qy)
        yaw_2 = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(yaw_1, yaw_2)



    def get_drone_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link_1', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w

            yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
            self.current_yaw = yaw

            return x, y, z
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("Ne mogu dohvatiti transformaciju između 'map' i 'base_link_1'")
            return None

    def timer_callback(self):
        if self.goal_reached:
            return

        if not self.path or self.current_target_index >= len(self.path):
            self.send_stop()
            return

        drone_pos = self.get_drone_position()
        if drone_pos is None:
            return


        px, py, pz = drone_pos
        goal = self.path[-1]

        # trenutačna ciljna točka na putanji
        target_point = PointStamped()
        target_point.header.frame_id = "map"
        target_point.point = self.path[self.current_target_index]

        try:
            trans = self.tf_buffer.lookup_transform('base_link_1', target_point.header.frame_id, rclpy.time.Time())
            target_transformed = do_transform_point(target_point, trans)
            dx = target_transformed.point.x
            dy = target_transformed.point.y
            dz = target_transformed.point.z
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("Transformacija ciljne točke nije dostupna")
            return

        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist < 0.1:
            self.get_logger().info(f"Točka {self.current_target_index} dosegnuta.")
            self.current_target_index += 1
            if self.current_target_index >= len(self.path):
                self.send_stop()
            return

        vel = min(self.Kp * dist, self.max_vel)

        vel_msg = Twist()
        vel_msg.linear.x = vel * dx / dist
        vel_msg.linear.y = vel * dy / dist
        vel_msg.linear.z = vel * dz / dist

        # kontrola yaw prema cilju
        desired_yaw = math.atan2(goal.y - py, goal.x - px)
        yaw_error = desired_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        vel_msg.angular.z = 0.0 * yaw_error #   regulacija kutne brzine, trenutačno 0 -> bez rotacije
        self.cmd_pub_drone.publish(vel_msg)


    def send_stop(self):
        stop_msg = Twist()
        self.cmd_pub_drone.publish(stop_msg)
        self.goal_reached = True
        self.get_logger().info("Putanja završena. Dron zaustavljen!")


    def stop_tracker_callback(self, msg):
        if msg.data:
            self.goal_reached = True 
            stop_msg = Twist()
            self.cmd_pub_drone.publish(stop_msg)
            self.get_logger().warn('Primljen signal za zaustavljanje drona!')


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerCrta()
    rclpy.spin(node)
    rclpy.shutdown()