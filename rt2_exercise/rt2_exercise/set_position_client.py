import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# from std_msgs.msg import Float32, Int32
from rt2_interfaces.action import MoveX

class SetPositionClient(Node):
    def __init__(self):
        super().__init__('set_position_client')
        # initialize action client
        self._action_client = ActionClient(self, MoveX, '/MoveX')

    def send_goal(self, x):
        goal = MoveX.Goal()
        goal.goal_x = x

        # wait for the server
        self._action_client.wait_for_server()

        # send goal and specify the feedback callback
        self._send_goal_future = self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        # define the goal response callback, which will be executed when the server responds
        # this allow us to know when the action is finished without "blocking" the code
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # 'future' stores the response of the server, and the first result we get is 'goal_handle'
        # it is a token that lets us track (and cancel) the goal, we use it to see if 
        # the goal has been accepted or not
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal refused')
            return
        self.get_logger().info('Goal accepted')

        # now we can check the final result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.remaining_x}')

        '''
        if self._cancel_sent or self._goal_handle is None:
            return
        # condition to cancel the goal
        if remaining < 0.01 and remaining > -0.01:
            self._cancel_sent = True
            self.get_logger().warn('Remaining distance less than 0.01 m, cancelling goal')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        '''

    # this notifies us when the action server has finished its job
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Translation to {result.final_x}: {result.success}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = SetPositionClient()
    action_client.send_goal(2.0)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()