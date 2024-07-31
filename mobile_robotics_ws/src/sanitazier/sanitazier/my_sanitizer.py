import rclpy
from rclpy.node import Node
#from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import numpy as np
import tf2_ros
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
dim=10
class SanitizerNode(Node):
    def __init__(self):
        super().__init__('sanitizer_node')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.areas_publisher = self.create_publisher(OccupancyGrid, 'sanitization_areas', 10)
        self.spread_energy_publisher = self.create_publisher(OccupancyGrid, 'energy', 10)
        self.regions = [(-dim,-dim,-dim,-dim),(dim,dim,dim,dim)]  # x y min x y max
        self.resolution = 0.15 # Define your resolution here
        self.energy_sampling = 0.1
        self.publish_sanitization_areas()
        self.create_timer(self.energy_sampling, self.spread_energy)
        self.create_timer(3.0, self.publish_sanitization_areas)
        self.create_timer(self.energy_sampling/10,self.get_robot_pose)
        self.create_timer(4.0,self.send_goal)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, 10)
        self.robot_pose = None
        self.init = True
        self.energy_data_real = None
        self.energy_data = None #for visualization
        self.minimum_energy = 0.01
        self.parse_text_file('/home/gabriele/mobile_robotics_ws/area_to_be_sanitized.txt')
    
    def laser_scan_callback(self, msg):
        self.laser_scan = msg
        
    def parse_text_file(self, file_path):
        with open(file_path, 'r', encoding='utf-8-sig') as file:
            for line in file:
                line = line.strip()  # remove newline characters
                if line == '/':  # If a line is '/', stop parsing
                    self.get_logger().info('Finished reading regions')
                    break
                elif line:  # if line is not empty
                    # remove the semicolon at the end of the line
                    line = line.rstrip(';')
                    # split by semicolon to get each set of coordinates
                    sets_of_coordinates = line.split(';')
                    for set_of_coordinates in sets_of_coordinates:
                        if set_of_coordinates:  # if set_of_coordinates is not empty
                            # split by comma to get the individual coordinates
                            coordinates = set_of_coordinates.split(',')
                            # convert to float and add to regions
                            self.regions.append(tuple(float(coordinate) for coordinate in coordinates if coordinate))
                            
    def publish_sanitization_areas(self):
        resolution = self.resolution

        x_size = int((max(region[2] for region in self.regions) - min(region[0] for region in self.regions)) / resolution)
        y_size = int((max(region[3] for region in self.regions) - min(region[1] for region in self.regions)) / resolution)
        x_min = min(region[0] for region in self.regions)
        x_max = max(region[2] for region in self.regions)
        y_min = min(region[1] for region in self.regions)
        y_max = max(region[3] for region in self.regions)
        x_center = (x_min + x_max) / 2.0
        y_center = (y_min + y_max) / 2.0

        grid = OccupancyGrid()
        grid.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        grid.info.resolution = resolution
        grid.info.width = x_size
        grid.info.height = y_size
        grid.info.origin.position.x = x_center - x_size * resolution / 2.0
        grid.info.origin.position.y = y_center - y_size * resolution / 2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (x_size * y_size) 

        for region in self.regions:
            x_min, y_min, x_max, y_max = region
            for x in np.arange(x_min, x_max, resolution):
                for y in np.arange(y_min, y_max, resolution):
                    x_index = int((x - grid.info.origin.position.x) / resolution)
                    y_index = int((y - grid.info.origin.position.y) / resolution)
                    grid.data[y_index * x_size + x_index] = 100
        self.grid = grid
        self.areas_publisher.publish(grid)

    def spread_energy(self):
        
        x_size = int((max(region[2] for region in self.regions) - min(region[0] for region in self.regions)) / self.resolution)
        y_size = int((max(region[3] for region in self.regions) - min(region[1] for region in self.regions)) / self.resolution)
        x_min = min(region[0] for region in self.regions)
        x_max = max(region[2] for region in self.regions)
        y_min = min(region[1] for region in self.regions)
        y_max = max(region[3] for region in self.regions)
        x_center = (x_min + x_max) / 2.0
        y_center = (y_min + y_max) / 2.0

        if self.init:
            self.energy_data = OccupancyGrid()
            self.energy_data.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
            self.energy_data.info.resolution = self.resolution
            self.energy_data.info.width = x_size
            self.energy_data.info.height = y_size
            self.energy_data.info.origin.position.x = x_center - x_size * self.resolution / 2.0
            self.energy_data.info.origin.position.y = y_center - y_size * self.resolution / 2.0
            self.energy_data.info.origin.orientation.w = 1.0
            self.energy_data.data = [0] * (x_size * y_size)
            self.energy_data_real = [0] * (x_size * y_size)

            self.init = False
        
        if self.robot_pose is None:
            return
        
        robot_x = self.robot_pose.transform.translation.x
        robot_y = self.robot_pose.transform.translation.y
        
        minimum_range = min(self.laser_scan.ranges)
        for x in np.arange(x_min, x_max, self.resolution):
            for y in np.arange(y_min, y_max, self.resolution):
                x_index = int((x - self.energy_data.info.origin.position.x) / self.resolution)
                y_index = int((y - self.energy_data.info.origin.position.y) / self.resolution)
                distance = (x - robot_x) ** 2 + (y - robot_y) ** 2
                if distance < minimum_range ** 2 and distance > 0.05:
                    self.energy_data_real[y_index * x_size + x_index] += 0.0003*self.energy_sampling/distance
                if self.energy_data_real[y_index * x_size + x_index] > self.minimum_energy:
                    self.energy_data.data[y_index * x_size + x_index] = 100

        self.spread_energy_publisher.publish(self.energy_data)

    def get_robot_pose(self):
        try:
            self.robot_pose = self.tf_buffer.lookup_transform('map', 'base_link',  rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            
            return
        
    def send_goal(self):
        possible_goals = []

        if self.robot_pose is None:
            return

        # Get robot position
        robot_x = self.robot_pose.transform.translation.x
        robot_y = self.robot_pose.transform.translation.y
        
        # Iterate over each cell in self.grid
        for i in range(len(self.grid.data)):
            # If the cell value is 100
            if self.grid.data[i] == 100:
                # Calculate corresponding index in self.energy_data
                j = i
                # Check the corresponding cell in self.energy_data
                if self.energy_data.data[j] != 100:
                    # Create PoseStamped object with cell's coordinates
                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.pose.position.x = (i % self.grid.info.width) * self.grid.info.resolution - dim
                    goal.pose.position.y = (i // self.grid.info.width) * self.grid.info.resolution - dim
                    goal.pose.orientation.w = 1.0
                    # Distance from robot to goal
                    distance = ((goal.pose.position.x - robot_x) ** 2 + (goal.pose.position.y - robot_y) ** 2) ** 0.5
                    # Add it to the list of possible goals
                    if distance > 0.5:
                        possible_goals.append((goal, distance))
        # Sort the list of possible goals based on distance from the robot
        possible_goals.sort(key=lambda x: x[1])
        if possible_goals:
            # Now you can safely access elements in possible_goals
            pose=possible_goals[0][0]
            print(f"Sending goal to: ({pose.pose.position.x}, {pose.pose.position.y})")
        else:
            print("No possible goals found.")
            return
        # Send the goal
        goal_msg = NavigateToPose.Goal()
        pose=possible_goals[0][0]
        goal_msg.pose = pose
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)
    node = SanitizerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()