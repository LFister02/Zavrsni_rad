import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import random
import heapq
import time
import csv
import os
import math


class AStarTest(Node):
    def __init__(self):
        super().__init__('a_star_nr_test')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.path_pub = self.create_publisher(Path, 'astar_path', 10)

        #   Definiranje naziva CSV file-a i spremanje u home direktorij
        self.declare_parameter("file_name", "ime_datoteke")
        self.file_name = self.get_parameter("file_name").get_parameter_value().string_value
        self.home = os.path.expanduser("~")
        self.filepath = os.path.join(self.home, self.file_name + ".csv") 

        # Velicina prostora, broj prepreka i maksimalna visina prepreka
        self.declare_parameter("grid_size_x", 100)
        self.declare_parameter("grid_size_y", 100)
        self.declare_parameter("grid_size_z", 6)

        x = self.get_parameter("grid_size_x").get_parameter_value().integer_value
        y = self.get_parameter("grid_size_y").get_parameter_value().integer_value
        z = self.get_parameter("grid_size_z").get_parameter_value().integer_value
        self.grid_size = (x, y, z)

        self.declare_parameter("num_obstacles", 400)
        self.num_obstacles = self.get_parameter("num_obstacles").get_parameter_value().integer_value

        self.declare_parameter("max_obstacle_height", 4)
        self.max_obstacle_height = self.get_parameter("max_obstacle_height").get_parameter_value().integer_value

        # Broj iteracija
        self.declare_parameter("wanted_iterations", 500)
        self.wanted_iterations = self.get_parameter("wanted_iterations").get_parameter_value().integer_value
        self.iteration = 0

        # Rezolucija
        self.resolution = 1.0
        
        # Pozicije starta
        self.start_x_pos = 2
        self.start_y_pos = 2
        self.start_z_pos = 1

        # Pozicije cilja
        self.goal_x_pos = x - 2
        self.goal_y_pos = y - 2
        self.goal_z_pos = 1

        # Zadavanje tipa heuristike
        self.declare_parameter("heuristic_type", "manhattan")      # manhattan / euclidean
        self.heuristic_type = self.get_parameter("heuristic_type").get_parameter_value().string_value

        # Zadavanje mogućnosti kretanja
        self.declare_parameter("kretanje", "dijagonalno")
        self.kretanje = self.get_parameter("kretanje").get_parameter_value().string_value    # jednostavno / dijagonalno (kretanje u 6 / 26 smjerova)

        # Zadavanje kazne s obzirom na promjenu visine
        self.declare_parameter("heuristic_gore", 1.0)
        self.declare_parameter("heuristic_isto", 1.0)
        self.declare_parameter("heuristic_dolje", 1.0)

        self.heuristic_gore = self.get_parameter("heuristic_gore").get_parameter_value().double_value     
        self.heuristic_isto = self.get_parameter("heuristic_isto").get_parameter_value().double_value       
        self.heuristic_dolje = self.get_parameter("heuristic_dolje").get_parameter_value().double_value     

        # Snimanje rezultata - csv tip datotoeke
        self.declare_parameter("save_results_csv", True)
        self.save_results_csv = self.get_parameter("save_results_csv").get_parameter_value().bool_value

        # Pomocne varijable
        self.sum_duration = 0
        self.path_not_f = 0

        # Nacin rada
        self.declare_parameter("work_type", "non_random")
        self.work_type = self.get_parameter("work_type").get_parameter_value().string_value     # work_types = [non_ranodm | random | point_click | random_height]

        if self.work_type == "non_random" or self.work_type == "random" or self.work_type.lower() == "random_height":
            self.timer_period = 1.0
            self.timer = self.create_timer(self.timer_period, self.run_astar_cycle)

        elif self.work_type == "point_click":
            self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
            self.start_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.start_callback, 10)
            self.goal_received = False
            self.goal = None
            self.start_received = False
            self.start = None
            
            self.occupied_voxels = set()
            self.buffer_zones = set()
            self.generate_obstacles()
            self.publish_obstacles()
            self.obstacle_buffer_zone()

        else:
            self.get_logger().warn("Nije zadan način rada! Pokušajte ponovno: [non_ranodm | random | point_click | random_height]")


    #   Generiraj prepreke 
    def generate_obstacles(self):
        if self.work_type.lower() == "non_random":
            counter = 0

            while counter < self.num_obstacles:
                x = random.randint(0, self.grid_size[0] - 1)
                y = random.randint(0, self.grid_size[1] - 1)

                if any ((x, y, z) == self.start or (x, y, z) == self.goal for z in range(self.max_obstacle_height)):
                    continue
                
                elif abs(x - self.start_x_pos) <= 1 and abs(y - self.start_y_pos) <= 1:
                    continue
                
                elif abs(x - self.goal_x_pos) <= 1 and abs(y - self.goal_y_pos) <= 1:
                    continue

                for z in range(self.max_obstacle_height):
                    if z < self.grid_size[2]:
                        self.occupied_voxels.add((x, y, z))
                counter += 1
            
        elif self.work_type.lower() == "random" or self.work_type.lower() == "point_click":
            for _ in range(self.num_obstacles):
                x = random.randint(0, self.grid_size[0] - 1)
                y = random.randint(0, self.grid_size[1] - 1)

                for z in range(0, self.max_obstacle_height):
                    if z < self.grid_size[2]:
                        self.occupied_voxels.add((x, y, z))

        elif self.work_type.lower() == "random_height":
            counter = 0
            while counter < self.num_obstacles:
                x = random.randint(0, self.grid_size[0] - 1)
                y = random.randint(0, self.grid_size[1] - 1)
                obs_height = random.randint(0, self.grid_size[2] - 1)

                if any ((x, y, z) == self.start or (x, y, z) == self.goal for z in range(self.max_obstacle_height)):
                    continue
                
                elif abs(x - self.start_x_pos) <= 1 and abs(y - self.start_y_pos) <= 1:
                    continue
                
                elif abs(x - self.goal_x_pos) <= 1 and abs(y - self.goal_y_pos) <= 1:
                    continue

                for z in range(obs_height):
                    if z < self.grid_size[2]:
                        self.occupied_voxels.add((x, y, z))
                counter += 1

    #   Sigurnosna zona oko prepreka
    def obstacle_buffer_zone(self):

        directions = [(dx, dy, dz)  for dx in [-1, 0, 1]
                                        for dy in [-1, 0, 1]
                                        for dz in [-1, 0, 1]
                                        if (dx, dy, dz) != (0, 0, 0)]

        for i in self.occupied_voxels:
            for dx, dy, dz in directions:
                nx, ny, nz = i[0] + dx, i[1] + dy, i[2] + dz
                if (0 <= nx < self.grid_size[0] and 0 <= ny < self.grid_size[1] and 0 <= nz < self.grid_size[2] and (nx, ny, nz) not in self.occupied_voxels and (nx, ny, nz) not in self.buffer_zones):
                    self.buffer_zones.add((nx, ny, nz))
        return None


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


    def random_free_cell(self):
        while True:
            cell = (
                random.randint(0, self.grid_size[0] - 1),
                random.randint(0, self.grid_size[1] - 1),
                random.randint(0, self.grid_size[2] - 1),
            )
            if cell not in self.occupied_voxels and cell not in self.buffer_zones:
                return cell


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
            if (0 <= nx < self.grid_size[0] and
                0 <= ny < self.grid_size[1] and
                0 <= nz < self.grid_size[2] and
                (nx, ny, nz) not in self.occupied_voxels and (nx, ny, nz) not in self.buffer_zones):
                neighbors.append((nx, ny, nz))
        return neighbors


    def heuristic(self, current, goal):
        dx = abs(current[0] - goal[0])
        dy = abs(current[1] - goal[1])
        dz = current[2] - goal[2]

        if self.heuristic_type.lower() == "manhattan":
            return dx + dy + abs(dz)
        
        elif self.heuristic_type.lower() == "euclidean":
            return math.sqrt(dx**2 + dy**2 + dz**2)
        

    # Cijena (g)
    def cost(self, current, neighbor):
        dx = neighbor[0] - current[0]
        dy = neighbor[1] - current[1]
        dz = neighbor[2] - current[2]

        distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        if dz > 0:
            height_penalty = self.heuristic_gore
        elif dz < 0:
            height_penalty = self.heuristic_dolje
        else:
            height_penalty = self.heuristic_isto

        return distance * height_penalty
        
    #   A* algoritam
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

                #   Spremanje rezultata
                if self.save_results_csv:
                    with open(self.filepath, "a", newline = "") as file:
                        writer = csv.writer(file)
                        writer.writerow([self.iteration, expanded_nodes, len(path), duration])

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


    def start_callback(self, msg):
        path_empty = []
        x = int((msg.pose.pose.position.x - self.resolution / 2) // self.resolution)
        y = int((msg.pose.pose.position.y - self.resolution / 2) // self.resolution)
        z = 0
        if (x, y, z) not in self.occupied_voxels:
            self.start = (x, y, z)
            self.start_received = True
            self.get_logger().info(f"Primljen start: {self.start}")
            self.publish_sphere(self.start, 100, (0.0, 1.0, 0.0)) 
            self.publish_path(path_empty)
        else:
            self.get_logger().warn("Odabrana početna točka je unutar prepreke!")
        
    def goal_callback(self, msg):
        x = int(msg.pose.position.x // self.resolution)
        y = int(msg.pose.position.y // self.resolution)
        z = 0

        if (x, y, z) in self.occupied_voxels:
            self.get_logger().warn(f"Ciljna ćelija ({x},{y},{z}) je prepreka!")
            return

        self.goal = (x, y, z)
        self.goal_received = True
        self.get_logger().info(f"Primljena nova ciljna točka: {self.goal}")
        self.publish_sphere(self.goal, 101, (1.0, 0.0, 0.0))  
        self.run_astar_with_new_goal()


    def run_astar_with_new_goal(self):
        if self.start_received and self.goal_received:
            self.get_logger().info('Pokrećem A* algoritam s ručno određenim točkama')
            path = self.a_star(self.start, self.goal)
            if path:
                self.publish_path(path)
                self.get_logger().info(f"Putanja pronađena!")
            else:
                self.get_logger().warn(f"Putanja nije pronađena!")

    #   Automatsko pokretanje nove pretrage - za testiranje
    def run_astar_cycle(self):
        if self.iteration < self.wanted_iterations:
            self.iteration += 1
            self.get_logger().info(f"#{self.iteration} iteracija.")
            self.occupied_voxels = set()
            self.buffer_zones = set()

            if self.work_type.lower() == "non_random" or self.work_type.lower() == "random_height":
                self.start = (self.start_x_pos, self.start_y_pos, self.start_z_pos)
                self.goal = (self.goal_x_pos, self.goal_y_pos, self.goal_z_pos)
                self.generate_obstacles()
                self.publish_obstacles()
                self.obstacle_buffer_zone()

            elif self.work_type.lower() == "random":
                self.generate_obstacles()
                self.publish_obstacles()
                self.obstacle_buffer_zone()
                self.start = self.random_free_cell() 
                self.goal = self.random_free_cell()

                while self.goal == self.start:
                    self.goal = self.random_free_cell()


            self.publish_sphere(self.start, 100, (1.0, 0.0, 0.0))
            self.publish_sphere(self.goal, 101, (0.0, 1.0, 0.0))

            path = self.a_star(self.start, self.goal)
            path_empty = []     
            if path:
                self.publish_path(path)
                self.get_logger().info(f"#{self.iteration} iteracija - Putanja je pronađena!")
                self.get_logger().info("")
            
            else:
                self.get_logger().warn(f"#{self.iteration} iteracija - Putanja nije pronađena!")
                self.publish_path(path_empty)  
                self.get_logger().info("")
                self.path_not_f += 1
            

        if self.iteration == self.wanted_iterations:
            self.get_logger().info(f"Prosjecno vrijeme pretraživanja tijekom {self.iteration} iteracija s razlicitim konfiguracijskim prostorom, ali istim koordinatama starta i cilja: {float(self.sum_duration/self.iteration):.4f} ms")
            self.get_logger().info(f"Velicina prostora: {self.grid_size}, Broj prepreka: {self.num_obstacles}, Visina prepreka: {self.max_obstacle_height}")
            self.get_logger().info(f"Heuristika za penjanje: {self.heuristic_gore}")
            self.get_logger().info(f"Heuristika za kretanje na istoj visini: {self.heuristic_isto}")
            self.get_logger().info(f"Heuristika za spustanje: {self.heuristic_dolje}")
            self.get_logger().info(f"Provedeno je {self.wanted_iterations} iteracija. Putanja nije pronađena {self.path_not_f} puta.")

            self.iteration += 1

        else:
            pass

    
def main():
    rclpy.init()
    node = AStarTest()
    rclpy.spin(node)
    rclpy.shutdown()
