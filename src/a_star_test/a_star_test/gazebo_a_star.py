import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import heapq
import csv
import time
import math
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class AStarGazebo(Node):
    def __init__(self):
        super().__init__('gazebo_a_star')  
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.occupied_voxels = set()
        self.buffer_zones = set()
        self.resolution = 0.05

        self.kretanje = 'dijagonalno'   # [jednostavno | dijagonalno]
        self.heuristic_type = 'euclidean'   # [manhattan | euclidean]

        self.heuristic_dolje = 1.0
        self.heuristic_isto = 1.0
        self.heuristic_gore = 1.0

        self.goal_received = False
        self.goal = None
        self.start_received = False
        self.start = None
        self.max_z = int(7.0 * self.resolution // self.resolution) # Ograničenje visine
        self.z_offset = 1 # Korekcija visine markera

        self.sum_duration = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.load_voxels('putanja_do_CSV_datoteke') # Potrebno postaviti
        self.publish_obstacles()
        self.obstacle_buffer_zone()
        

    def load_voxels(self, filename):
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x = int(float(row['x']) / self.resolution)
                    y = int(float(row['y']) / self.resolution)
                    z = int((float(row['z']) / self.resolution) + self.z_offset)
                    self.occupied_voxels.add((x, y, z))
            self.get_logger().info(f"Učitano {len(self.occupied_voxels)} voxela iz {filename}.")
        
        except FileNotFoundError:
            self.get_logger().error(f"Datoteka {filename} nije pronađena!")
            
        except Exception as e:
            self.get_logger().error(f"Greška prilikom učitavanja CSV-a: {e}")

    
    def publish_obstacles(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = self.resolution
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.5, 0.1, 0.8, 1.0

        for x, y, z in self.occupied_voxels:
            pt = Point()
            pt.x = x * self.resolution + self.resolution / 2
            pt.y = y * self.resolution + self.resolution / 2
            pt.z = z * self.resolution + self.resolution / 2
            marker.points.append(pt)

        self.marker_pub.publish(marker)


    def obstacle_buffer_zone(self):

        directions = [(dx, dy, dz)  for dx in [-1, 0, 1]
                                        for dy in [-1, 0, 1]
                                        for dz in [-1, 0, 1]
                                        if (dx, dy, dz) != (0, 0, 0)]

        for i in self.occupied_voxels:
            for dx, dy, dz in directions:
                nx, ny, nz = i[0] + dx, i[1] + dy, i[2] + dz
                if ((nx, ny, nz) not in self.occupied_voxels and (nx, ny, nz) not in self.buffer_zones):
                    self.buffer_zones.add((nx, ny, nz))
        return None


    def publish_sphere(self, cell, marker_id, color):
        x, y, z = cell
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = self.resolution
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0
        marker.pose.position.x = x * self.resolution + self.resolution / 2
        marker.pose.position.y = y * self.resolution + self.resolution / 2
        marker.pose.position.z = z * self.resolution + self.resolution / 2
        self.marker_pub.publish(marker)


    def get_neighbors(self, node):

        if self.kretanje.lower() == "jednostavno":
            directions = [
                (1, 0, 0), (-1, 0, 0),
                (0, 1, 0), (0, -1, 0),
                (0, 0, 1), (0, 0, -1),
            ]

        elif self.kretanje.lower() == "dijagonalno":
            directions = [(dx, dy, dz)  for dx in [-1, 0, 1]
                                        for dy in [-1, 0, 1]
                                        for dz in [-1, 0, 1]
                                        if (dx, dy, dz) != (0, 0, 0)]

        neighbors = []
        for dx, dy, dz in directions:
            nx, ny, nz = node[0] + dx, node[1] + dy, node[2] + dz
            if (0 <= nz < self.max_z and ((nx, ny, nz) not in self.occupied_voxels and (nx, ny, nz) not in self.buffer_zones)):
                neighbors.append((nx, ny, nz))
        return neighbors


    def heuristic(self, current, goal):
        dx = abs(current[0] - goal[0])
        dy = abs(current[1] - goal[1])
        dz = abs(current[2] - goal[2])

        if self.heuristic_type.lower() == "manhattan":
            return dx + dy + dz
        
        elif self.heuristic_type.lower() == "euclidean":
            return math.sqrt(dx**2 + dy**2 + dz**2)
        

    # Računanje cijene za prelezak iz trenutnog čvora na susjedni
    def cost(self, current, neighbor):
        dx = neighbor[0] - current[0]
        dy = neighbor[1] - current[1]
        dz = neighbor[2] - current[2]

        distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        # Kazna za penjanje
        if dz > 0:
            height_penalty = self.heuristic_gore
        elif dz < 0:
            height_penalty = self.heuristic_dolje
        else:
            height_penalty = self.heuristic_isto

        return distance * height_penalty
        

    def a_star(self, start, goal):
        start_time = time.time()
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start, goal), start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        open_set_member = set()
        open_set_member.add(start)

        closed_set = set()
        expanded_nodes = 0

        while open_set:
            current_f_score, current = heapq.heappop(open_set)
            open_set_member.remove(current)
            expanded_nodes += 1

            if current == goal:
                path = self.reconstruct_path(came_from, current)
                duration = (time.time() - start_time) * 1000
                self.sum_duration += duration

                
                self.get_logger().info(f"Pretraga je gotova:")
                self.get_logger().info(f"Broj proširenih čvorova: {expanded_nodes}, duljina putanje: {len(path)}, trajanje pretrage: {duration:.4f} ms")
                self.get_logger().info(f"Ukupno trajanje pretraživanja: {self.sum_duration:.4f} ms")
                return path
            
            closed_set.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + self.cost(current, neighbor)  
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)

                    if neighbor not in open_set_member:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_member.add(neighbor)

        duration = (time.time() - start_time) * 1000
        self.get_logger().warn(f"Pretraga nije uspjela. Trajanje: {duration:.4f} milisekundi, čvorova prošireno: {expanded_nodes}")

        return None


    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = "map"
        for x, y, z in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x * self.resolution + self.resolution / 2
            pose.pose.position.y = y * self.resolution + self.resolution / 2
            pose.pose.position.z = z * self.resolution + self.resolution / 2
            msg.poses.append(pose)
        self.path_pub.publish(msg)


    def get_start_from_slam(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link_1', rclpy.time.Time())
            x = int(trans.transform.translation.x // self.resolution)
            y = int(trans.transform.translation.y // self.resolution)
            z = int(trans.transform.translation.z // self.resolution)


            if (x, y, z) in self.occupied_voxels:
                self.get_logger().warn(f"Start pozicija ({x},{y},{z}) je unutar prepreke.")
                return None

            self.get_logger().info(f"Start pozicija dobivena: ({x},{y},{z})")
            return (x, y, z)

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("Ne mogu dohvatiti transformaciju 'map' -> 'dron'")
            return None
        
    def goal_callback(self, msg):
        try:
            x = int(msg.pose.position.x // self.resolution)
            y = int(msg.pose.position.y // self.resolution)
            z = int(3.0 * self.resolution // self.resolution)

            if (x, y, z) in self.occupied_voxels:
                self.goal_received = False 
                self.get_logger().warn(f"Ciljna točka ({x},{y},{z}) je u prepreci!")
                return

            self.goal = (x, y, z)
            self.goal_received = True
            self.get_logger().info(f"Primljena nova ciljna točka: {self.goal}")
            self.publish_sphere(self.goal, 101, (1.0, 0.0, 0.0))
            self.run_astar_with_new_goal()
        except Exception as e:
            self.get_logger().error(f"Greška u goal_callback: {e}")


    def run_astar_with_new_goal(self):
        self.start = self.get_start_from_slam()
        self.start_received = True
        if self.start_received and self.goal_received:
            self.get_logger().info('Pokrećem A* algoritam.')
            path = self.a_star(self.start, self.goal)
            if path:
                self.publish_path(path)
                self.get_logger().info(f"Putanja pronađena!")
            else:
                self.get_logger().warn(f"Putanja nije pronađena!")

        


def main(args=None):
    rclpy.init(args=args)
    node = AStarGazebo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()