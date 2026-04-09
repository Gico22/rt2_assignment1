import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rt2_interfaces.action import MoveX
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class SetPositionServer(Node):
    def __init__(self):
        super().__init__('set_position_server')

        # ReentrantCallbackGroup allows callbacks to run concurrently
        self.cb_group = ReentrantCallbackGroup()

        # initialize action server as a standard service
        self._action_server = ActionServer(
            self, MoveX, '/MoveX', 
            self.execute_callback, 
            callback_group=self.cb_group
        )

        # velocity publisher
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # x position subscriber
        self.pos_subscriber = self.create_subscription(
            Odometry, '/odom', 
            self.get_position_callback, 10,
            callback_group=self.cb_group
        )

        self.vel_x = Twist()
        self.stop = Twist()
        self.position_x = 0.0

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = MoveX.Feedback()
        goal = goal_handle.request.goal_x

        # set direction
        self.vel_x.linear.x = 0.5 if self.position_x < goal else -0.5
        
        # move the robot
        while abs(self.position_x - goal) > 0.1:
            self.vel_publisher.publish(self.vel_x)
            feedback_msg.remaining_x = abs(self.position_x - goal)
            # self.get_logger().info(f'Feedback: remaining distance = {feedback_msg.remaining_x}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        # stop the robot
        self.vel_publisher.publish(self.stop)

        goal_handle.succeed()
        result = MoveX.Result()
        result.success = True
        result.final_x = self.position_x
        return result
    
    def get_position_callback(self, msg):
        self.position_x = msg.pose.pose.position.x

def main(args=None):
    rclpy.init(args=args)
    set_position_server = SetPositionServer()
    executor = MultiThreadedExecutor()  # use this instead of rclpy.spin()
    executor.add_node(set_position_server)
    executor.spin()

if __name__ == '__main__':
    main()