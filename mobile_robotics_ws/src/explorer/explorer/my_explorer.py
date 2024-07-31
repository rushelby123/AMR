import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import random 
import tf2_ros
import tf2_geometry_msgs
from threading import Timer
import math
import random
import time

class MapSubscriber(Node):

    def __init__(self):
        super().__init__('map_subscriber')
        self.marker_pub = self.create_publisher(MarkerArray, 'frontiers', 1)
        self.unwanted_mark_pub = self.create_publisher(MarkerArray, 'unwanted_frontiers', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 1)
        self.subscription = self.create_subscription(OccupancyGrid,'/map',self.listener_callback,10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.subscription
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.moving_to_goal = False
        self.goal_msg = None
        self.map_msg = None
        self.frontiers = None
        self.robot_pose = None
        self.robot_pose_some_time_ago = None  
        self.last_time_checked = time.time()  
        self.unwanted_frontiers = []


    def listener_callback(self, msg):
        self.map_msg = msg

        #get the robot pose
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_pose = transform.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('TF2 error: %s' % str(e))
            return None
        
        # Check if the robot reached the goal position
        if self.robot_pose is not None and self.moving_to_goal:
            # Calculate the distance between the robot's pose and the target pose
            dx = self.robot_pose.x - self.goal_msg.pose.position.x
            dy = self.robot_pose.y - self.goal_msg.pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            #print(f"distance from target:{distance}")
            if distance<0.5:
                # Cancel the timers when the goal is reached
                print("Goal reached")
                self.timer.cancel()
                self.moving_to_goal = False

            #check if the robot is moving 
            if self.robot_pose_some_time_ago==None:
                self.robot_pose_some_time_ago=self.robot_pose
            if time.time() - self.last_time_checked >= 7: #5 second in this case
                # Update the robot's position 5 seconds ago and the last checked time
                self.last_time_checked = time.time()
                dx = self.robot_pose.x - self.robot_pose_some_time_ago.x
                dy = self.robot_pose.y - self.robot_pose_some_time_ago.y
                distance_moved_in_last_5s = math.sqrt(dx**2 + dy**2)
                if distance_moved_in_last_5s<0.1:
                    #print(f"Distance moved in the last few seconds: {distance_moved_in_last_5s}")
                    self.timer.cancel()
                    self.cancel_goal()
                    self.moving_to_goal = False
                self.robot_pose_some_time_ago = self.robot_pose

        # Only send a new goal if the robot is not currently moving towards a goal
        if not self.moving_to_goal:
            print('sending goal...')

            #update the frontiers 
            self.frontiers = find_frontiers(msg)
            frontiers=self.frontiers
            print(f"unwanted frontiers: {len(self.unwanted_frontiers)}")
            #remove the unwanted frontiers from the frontiers list
            if len(self.unwanted_frontiers)>0:
                # create a list of frontiers in map frame
                frontiers_in_map_frame = []
                new_frontiers = []
                for i in range(len(frontiers)):
                    frontiers_in_map_frame.append((frontiers[i][0] * msg.info.resolution + msg.info.origin.position.x, frontiers[i][1] * msg.info.resolution + msg.info.origin.position.y))
                #check if among the frontiers there are the unwanted ones           
                for i in range(len(frontiers)):
                    frontier = frontiers_in_map_frame[i]
                    is_close_to_unwanted = any(
                        math.sqrt((frontier[0]-unwanted[0])**2+(frontier[1]-unwanted[1])**2) <= 0.2
                        for unwanted in self.unwanted_frontiers
                    )
                    if not is_close_to_unwanted:
                        new_frontiers.append(frontiers[i])
                self.frontiers = new_frontiers
                frontiers = self.frontiers

            #check the frontiers of the map and publish them
            print('Found %d frontiers' % len(frontiers))
            markers = MarkerArray()
            for i, (x, y) in enumerate(frontiers):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = i
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.pose.position.x = x * msg.info.resolution + msg.info.origin.position.x
                marker.pose.position.y = y * msg.info.resolution + msg.info.origin.position.y
                marker.pose.position.z = 0.1
                marker.pose.orientation.w = 1.0
                marker.scale.x = msg.info.resolution
                marker.scale.y = msg.info.resolution
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                markers.markers.append(marker)
            self.marker_pub.publish(markers)

            # Create a goal message
            goal_msg = NavigateToPose.Goal()
            goal_pose = PoseStamped()
            robot_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'

            if frontiers:
                # Initialize the minimum distance and the selected index
                min_distance = float('inf')
                selected_index = -1

                # Iterate over the frontiers
                random.shuffle(frontiers) #shuffle the frontiers to avoid always selecting the same one
                for i in range(len(frontiers)):
                    # Calculate the distance to the frontier
                    dx = float(frontiers[i][0]) * msg.info.resolution + msg.info.origin.position.x- self.robot_pose.x
                    dy = float(frontiers[i][1]) * msg.info.resolution + msg.info.origin.position.y - self.robot_pose.y
                    distance = math.sqrt(dx**2 + dy**2)

                    # If the distance is less than 5 meters and less than the current minimum, select this frontier
                    if distance < 3.0 and distance < min_distance:
                        min_distance = distance
                        selected_index = i
                        print(f"distance from target:{distance}")
                        break

                # If no frontier was found within 5 meters, select a random one
                if selected_index == -1:
                    selected_index = random.randint(0, len(frontiers) - 1)
                self.selected_index = selected_index

                # Set the goal pose to the selected frontier
                goal_pose.pose.position.x = float(frontiers[selected_index][0]) * msg.info.resolution + msg.info.origin.position.x
                goal_pose.pose.position.y = float(frontiers[selected_index][1]) * msg.info.resolution + msg.info.origin.position.y
                goal_pose.pose.orientation.w = 1.0  
                goal_msg.pose = goal_pose
                self.action_client.wait_for_server()
                self.send_goal_future = self.action_client.send_goal_async(goal_msg)
                self.goal_handle = self.send_goal_future.result()
                goal_status = self.send_goal_future.result()
                self.goal_status = goal_status
                self.goal_msg=goal_pose

                # Set the pose of the robot
                robot_pose.pose.position.x = self.robot_pose.x
                robot_pose.pose.position.y = self.robot_pose.y
                robot_pose.pose.orientation.w = 1.0  

                # Start a timer that will cancel the goal after some seconds
                self.timer = Timer(120.0, self.cancel_goal)
                self.timer.start()

                # Publish the goal pose
                self.goal_pub.publish(goal_pose)

                # Publish the robot pose
                self.robot_pose_pub.publish(robot_pose)

                # Set the moving_to_goal flag to True
                self.moving_to_goal = True

            else:
                self.get_logger().info('No frontiers found')
                self.moving_to_goal = False
        
                
    def get_robot_pose(self):
        # Create a buffer and a listener
        tf_buffer = tf2_ros.Buffer()

        # Get the transform from the map frame to the base_link frame
        try:
            transform = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('TF2 error: %s' % str(e))
            return None

        # Convert the transform to a Pose message
        robot_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(), transform)

        return robot_pose
    
    def cancel_goal(self):
        # Add the selected frontier to the list of unwanted frontiers
        self.timer.cancel()
        print("Cancelling goal...")

        # we need to add the selected frontier to unwanted frontiers, 
        # since it couldn't be reached be reached

        #transform the selected frontier to the map frame, NOTE THAT THE SIZE OF THE MAP IS CHANGING EVERY UPDATE!
        selected_frontier = self.frontiers[self.selected_index]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = float(selected_frontier[0]) * self.map_msg.info.resolution + self.map_msg.info.origin.position.x
        goal_pose.pose.position.y = float(selected_frontier[1]) * self.map_msg.info.resolution + self.map_msg.info.origin.position.y
        goal_pose.pose.orientation.w = 1.0
        
        # append the selected frontier to the unwanted frontiers
        transformed_frontier = [selected_frontier[0] * self.map_msg.info.resolution  + self.map_msg.info.origin.position.x , selected_frontier[1] * self.map_msg.info.resolution + self.map_msg.info.origin.position.y]
        self.unwanted_frontiers.append(transformed_frontier)

        # publish the orange marker of the unwanted frontiers
        markers = MarkerArray()
        for i, (x, y) in enumerate(self.unwanted_frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position.x = x 
            marker.pose.position.y = y 
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.map_msg.info.resolution
            marker.scale.y = self.map_msg.info.resolution
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            markers.markers.append(marker)
        self.unwanted_mark_pub.publish(markers)

        #cancel the goal
        self.moving_to_goal = False
        
        
def find_frontiers(map_msg):
    # This function returns a list of frontiers in the map
    # Note that x and y are the pixel coordinates of the map
    # Convert the map to a 2D numpy array for easier processing
    map_array = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

    # Initialize an empty list to hold the frontier cells
    frontiers = []

    # Iterate over all cells in the map
    for y in range(map_msg.info.height):
        for x in range(map_msg.info.width):
            may_be_frontier = False
            may_be_wall = False
            # Create a Kernel that check if the cell is known and not occupied
            if map_array[y, x] >= 20:
                # Check the neighboring cells
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = x + dx, y + dy
                    # If the neighbor is within the map bounds and is unknown, add the current cell to the frontiers
                    if (0 <= nx < map_msg.info.width and 0 <= ny < map_msg.info.height and map_array[ny, nx] <= 1):
                        #raise a flag 
                        may_be_frontier = True
                        break  
                #remove from the frontiers the cells that are too close to the wall
                if may_be_frontier:
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, ny = x + dx, y + dy
                        # If the neighbor is within the map bounds and is occupied, add the current cell to the walls
                        if (0 <= nx < map_msg.info.width and 0 <= ny < map_msg.info.height and map_array[ny, nx] > 50):
                            may_be_wall = True
                            break    
                    if not may_be_wall:
                        frontiers.append((x, y))

    return frontiers

        
def main(args=None):
    rclpy.init(args=args)

    map_subscriber = MapSubscriber()

    rclpy.spin(map_subscriber)

    map_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()