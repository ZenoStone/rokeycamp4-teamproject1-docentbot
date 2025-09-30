import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from model_dd_interfaces.action import DetectAndMeasure

class DetectAndMeasureActionClient(Node):
    def __init__(self):
        super().__init__('detect_and_measure_action_client')
        self._action_client = ActionClient(self, DetectAndMeasure, 'detect_and_measure')

    def send_goal(self):
        goal_msg = DetectAndMeasure.Goal()

        self._action_client.wait_for_server()
        self.get_logger().info('Action server available. Sending goal.')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'[목표 달성] : 최종 측정 거리 {result.final_depth:.2f}m')
        else:
            self.get_logger().warn('[목표 실패] : 최종적으로 거리를 측정하지 못했습니다.')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'[피드백] : 현재 측정 거리 {feedback_msg.feedback.current_depth:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    action_client = DetectAndMeasureActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
