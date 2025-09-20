import rclpy
from std_msgs.msg import Bool
from rclpy.node import Node 
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import math 
from sensor_msgs.msg import Range
from tf2_geometry_msgs import do_transform_point
import time



class PathFollowerDynamic(Node):
    def __init__(self):
        super().__init__('path_follower_dynamic')

        self.path_sub = self.create_subscription(Path, '/astar_path', self.path_callback, 10)
        self.cmd_pub_drone = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.stop_tracker_sub = self.create_subscription(Bool, '/drone1/stop_tracker', self.stop_tracker_callback, 10)
        self.range_sensor_sub = self.create_subscription(Range, '/range_sensor', self.sensor_callback,10)
        self.new_obs_pub = self.create_publisher(PointStamped, '/new_obstacle', 10)

        self.replan = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = []  
        self.current_target_index = 0
        self.goal_reached = False   

        self.Kp = 0.4      
        self.max_vel = 0.1     
        self.timer = self.create_timer(0.1, self.timer_callback) 

        self.obstacle_detected = False
        self.range = None
        self.safe_range = 1.0

        self.resolution = 0.05



    def path_callback(self, msg):
        self.path = [pose.pose.position for pose in msg.poses]  
        self.current_target_index = 0  
        self.goal_reached = False 
        self.get_logger().info(f"Primljena je nova putanja sa: ({len(self.path)} točaka).")



    def get_drone_position(self):
        try:
            transformation = self.tf_buffer.lookup_transform('map', 'base_link_1', rclpy.time.Time())
            x = transformation.transform.translation.x
            y = transformation.transform.translation.y
            z = transformation.transform.translation.z
            return x, y, z
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("Nema transformacije između 'map' i 'base_link_1'")
            return None

    def timer_callback(self):
        if self.goal_reached:
            return
        
        elif self.obstacle_detected:
            self.safe_stop()
            time.sleep(3.0)
            self.obstacle_detected = True
            self.replan = True


        elif not self.path or self.current_target_index >= len(self.path):
            self.send_stop()
            return

        drone_pos = self.get_drone_position()
        if drone_pos is None:
            return

        target_point = PointStamped()
        target_point.header.frame_id = "map"
        target_point.point = self.path[self.current_target_index]

        try:
            transformation = self.tf_buffer.lookup_transform('base_link_1', target_point.header.frame_id, rclpy.time.Time())
            target_transformed = do_transform_point(target_point, transformation)
            dx = target_transformed.point.x
            dy = target_transformed.point.y
            dz = target_transformed.point.z
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("Transformacija točke nije dostupna")
            return

        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        threshold = 0.075
        if dist < threshold:
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

        self.cmd_pub_drone.publish(vel_msg)

    def send_stop(self):
        stop_msg = Twist()
        self.cmd_pub_drone.publish(stop_msg)
        self.goal_reached = True
        self.get_logger().info("Misija završena. Dron zaustavljen!")

    def stop_tracker_callback(self, msg):
        if msg.data:
            self.goal_reached = True 
            stop_msg = Twist()
            self.cmd_pub_drone.publish(stop_msg)
            self.get_logger().warn('Primljen signal za zaustavljanje drona!')

    def safe_stop(self):
        stop_msg = Twist()
        self.cmd_pub_drone.publish(stop_msg)

    #   Slusanje informacija sa senzora
    def sensor_callback(self, msg):
        self.range = msg.range
        if self.range <= self.safe_range and not self.obstacle_detected and not self.replan:
            self.get_logger().warn("Prepreka detektirana - Sigurnosno zaustavljanje!")
            self.obstacle_detected = True

        elif self.range <= self.safe_range and self.replan:
            obs_pos = PointStamped()
            obs_pos.header.frame_id = "base_link_1"
            obs_pos.header.stamp = self.get_clock().now().to_msg()
            obs_pos.point.x = self.range * 0.15 # Korekcija mjerila Gazebo - Rviz
            obs_pos.point.y = 0.0
            obs_pos.point.z = 0.0

            try:
                transformation = self.tf_buffer.lookup_transform('map', obs_pos.header.frame_id, rclpy.time.Time())
                target_transformed = do_transform_point(obs_pos, transformation)
                dx = target_transformed.point.x
                dy = target_transformed.point.y
                dz = target_transformed.point.z
            except (LookupException, ConnectivityException, ExtrapolationException):
                    self.get_logger().warn("Transformacija točke nije dostupna")
                    return

            pos_msg = PointStamped()
            pos_msg.header.frame_id = "map"
            obs_pos.header.stamp = self.get_clock().now().to_msg()
            pos_msg.point.x = dx
            pos_msg.point.y = dy
            pos_msg.point.z = dz
            self.new_obs_pub.publish(pos_msg)
            self.obstacle_detected = False



        elif self.range > self.safe_range:
            self.replan = False
            self.obstacle_detected = False

        else:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerDynamic()
    rclpy.spin(node)
    rclpy.shutdown()