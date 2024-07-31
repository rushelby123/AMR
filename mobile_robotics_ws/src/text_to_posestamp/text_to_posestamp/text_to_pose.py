from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
import rclpy
import time
from std_msgs.msg import Int32

class TextToPoseNode(Node):
    def __init__(self):
        super().__init__('text_to_pose')
        self.file_path = '/home/gabriele/mobile_robotics_ws/go_to.txt'
        self.poses = self.parse_text_file(self.file_path)
        qos_profile = QoSProfile(depth=100)
        self.pose_count_publisher = self.create_publisher(Int32, 'pose_count_topic', qos_profile)
        self.publisher = self.create_publisher(PoseStamped, 'pose_topic', qos_profile)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.pose_count_publisher.publish(Int32(data=len(self.poses)))  

        for pose in self.poses:
            #self.get_logger().info(str(pose))
            self.publisher.publish(pose)

    def parse_text_file(self, file_path):
        with open(file_path, 'r') as file:
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
                                      
def main(args=None):
    rclpy.init(args=args)
    node = TextToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()