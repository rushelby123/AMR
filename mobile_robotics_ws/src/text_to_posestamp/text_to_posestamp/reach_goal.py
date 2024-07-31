import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import asyncio
import time
from std_msgs.msg import Int32
from rclpy.duration import Duration

class NavigateToPoseClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_poses = []  # List of goal poses
        self.number_of_poses = 0  # Initialize number_of_poses
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(PoseStamped,'pose_topic',self.pose_callback,qos_profile)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_count_subscription = self.create_subscription(Int32, 'pose_count_topic', self.pose_count_callback, qos_profile)  

    def pose_count_callback(self, msg):
        self.number_of_poses = msg.data
        #self.get_logger().info('Received pose count: %s' % msg.data)
        
    def pose_callback(self, msg):
        if not any(pose.pose.position == msg.pose.position and pose.pose.orientation == msg.pose.orientation for pose in self.goal_poses):
            self.goal_poses.append(msg)
        if len(self.goal_poses) == self.number_of_poses:
            self.run()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info('Received feedback: {0}'.format(feedback))

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        goal_reached=False
        while not goal_reached:
            #check the robot's distance from the goal
            goal_pose_x = pose.pose.position.x
            goal_pose_y = pose.pose.position.y

            rclpy.spin_once(self)

            try:
                robot_pose = self.tf_buffer.lookup_transform('map', 'base_link',  rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error('Could not get robot pose: %s' % e)
                return
            
            robot_pose_x = robot_pose.transform.translation.x
            robot_pose_y = robot_pose.transform.translation.y

            distance = ((goal_pose_x - robot_pose_x)**2 + (goal_pose_y - robot_pose_y)**2)**0.5
            print('Distance from goal: ', distance)

            if distance < 0.3:
                goal_reached=True
                self.get_logger().info('Goal reached')


    def run(self):
        self.get_logger().info('Received poses: %s' % len(self.goal_poses))
        tf_recived=False
        while tf_recived==False:
            try:
                robot_pose = self.tf_buffer.lookup_transform('map', 'base_link',  rclpy.time.Time())
                tf_recived=True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error('Could not get robot pose: %s' % e)
                continue
        for pose in self.goal_poses:
            self.get_logger().info('Taking action...')
            self.send_goal(pose)
        self.get_logger().info('Finished...')

def main(args=None):
    rclpy.init(args=args)
    navigate_to_pose_client = NavigateToPoseClient()
    rclpy.spin(navigate_to_pose_client)
    navigate_to_pose_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()