import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

class SingleUltrasonicSensorSim(Node):
    def __init__(self):
        super().__init__('scan_to_range')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.range_pub = self.create_publisher(Range, '/range_sensor', 10)
        
    
    def lidar_callback(self, msg: LaserScan):
        self.detected_ranges = []
        angle_min = -0.26
        angle_max = 0.26
        
        # Broj zraka za cijeli scan
        ray_num = len(msg.ranges)
        
        # Izračunaj indekse odgovarajućih kutova
        angle_increment = msg.angle_increment
        min_index = max(0, int((angle_min - msg.angle_min) / angle_increment))
        max_index = min(ray_num - 1, int((angle_max - msg.angle_min) / angle_increment))
        
        #   Simulacija range senzora - koriste se samo podaci unutar definiranog kuta cca 30° (HC-SR04)
        for r in msg.ranges[min_index:max_index + 1]:
            if msg.range_min <= r <= msg.range_max:
                self.detected_ranges.append(r)
            else:
                continue
        
        if self.detected_ranges:
            measured_range = min(self.detected_ranges)

        else:
            measured_range = msg.range_max

        
        # Izrada Range poruke
        range_msg = Range()
        range_msg.header.stamp = msg.header.stamp
        range_msg.header.frame_id = 'ultrasonic_link'
        range_msg.field_of_view = 0.52
        range_msg.min_range = msg.range_min
        range_msg.max_range = msg.range_max
        range_msg.range = float(measured_range)
        range_msg.radiation_type = Range.ULTRASOUND
        
        # Objavi Range poruku
        self.range_pub.publish(range_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = SingleUltrasonicSensorSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
