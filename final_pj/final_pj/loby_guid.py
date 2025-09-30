
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from std_msgs.msg import String
from my_interfaces.srv import GallerySelect
from my_interfaces.action import DetectAndMeasure

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

import time

class LobyGuidNode(Node):
    """
    LobyGuidNode는 TurtleBot4 로봇을 사용하여 사용자를 지정된 갤러리 위치로 안내하는 메인 제어 노드입니다.
    """
    def __init__(self):
        super().__init__('loby_guid_node')

        # 웨이포인트 정의
        self.waypoints = {
            '1': {'wp1': [-2.9106, -0.0541], 'goal': [-4.6673, -0.5359]},
            '2': {'wp1': [-3.4332, -0.0504], 'goal': [-6.0280, -0.2753]},
            '3': {'wp1': [-1.2520, 1.3543], 'goal': [-2.6589, 1.5903]},
            '4': {'wp1': [-3.4195, 1.3848], 'goal': [-5.4236, 1.7120]}
        }
        self.home_pose_coords = [-0.5457, 0.1060]

        # ROS2 시스템 초기화
        self.navigator = TurtleBot4Navigator()
        self.gallery_select_client = self.create_client(GallerySelect, '/robot6/gallery_select_service')
        self.detection_action_client = ActionClient(self, DetectAndMeasure, 'detect_and_measure')
        self.goal_publisher = self.create_publisher(String, 'robot6_goal', QoSProfile(depth=10))

        self.get_logger().info("Loby Guid Node가 시작되었습니다. 서비스 서버를 기다립니다.")
        self.call_gallery_select_service()

    def call_gallery_select_service(self):
        """
        갤러리 선택 서비스 서버에 연결을 시도하고, 연결되면 서비스를 호출합니다.
        """
        while not self.gallery_select_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('갤러리 선택 서비스가 아직 사용 가능하지 않습니다. 다시 시도합니다...')
        
        self.get_logger().info("갤러리 선택 서비스 서버에 성공적으로 연결되었습니다.")
        req = GallerySelect.Request()
        future = self.gallery_select_client.call_async(req)
        future.add_done_callback(self.gallery_select_callback)

    def gallery_select_callback(self, future):
        """
        갤러리 선택 서비스로부터 응답을 받으면 호출되는 콜백 함수입니다.
        """
        try:
            response = future.result()
            self.get_logger().info(f"사용자로부터 목적지 '{response.gallery_choice}'를 수신했습니다.")
            self.start_guidance(response.gallery_choice)
        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류 발생: {e}')

    def start_guidance(self, destination):
        """
        수신된 목적지를 기반으로 안내 프로세스를 시작합니다.
        """
        if destination not in self.waypoints:
            self.get_logger().error(f"알 수 없는 목적지 '{destination}'입니다. 안내를 종료합니다.")
            return

        target_waypoints = self.waypoints[destination]
        
        # 1. 중간 지점(wp1)으로 이동
        self.get_logger().info(f"중간 지점 {target_waypoints['wp1']}으로 이동을 시작합니다.")
        wp1_pose = self.navigator.getPoseStamped(target_waypoints['wp1'], TurtleBot4Directions.NORTH)
        self.navigator.goToPose(wp1_pose)
        
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)

        self.get_logger().info("중간 지점에 도착했습니다. 180도 회전합니다.")
        self.navigator.spin(3.14159) # 180도 회전
        
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)

        # 2. 사용자 인식 액션 요청
        self.get_logger().info("사용자 인식을 시작합니다.")
        self.send_detection_goal(target_waypoints)

    def send_detection_goal(self, target_waypoints):
        """
        detect_and_measure 액션 서버에 목표를 전송합니다.
        """
        self.detection_action_client.wait_for_server()
        goal_msg = DetectAndMeasure.Goal()
        
        self.get_logger().info("Detect and Measure 액션 서버에 목표를 전송합니다.")
        send_goal_future = self.detection_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        send_goal_future.add_done_callback(lambda future: self.get_result_callback(future, target_waypoints))

    def feedback_callback(self, feedback_msg):
        """
        액션 서버로부터 피드백을 수신했을 때 호출됩니다.
        """
        self.get_logger().info(f"피드백 수신: {feedback_msg.feedback.status}")

    def get_result_callback(self, future, target_waypoints):
        """
        액션 실행이 완료되면 호출됩니다.
        """
        result = future.result().result
        if result.success:
            self.get_logger().info("사용자 인식 성공! 최종 목적지로 이동을 시작합니다.")
            # 3. 최종 목적지(goal)로 이동
            goal_pose = self.navigator.getPoseStamped(target_waypoints['goal'], TurtleBot4Directions.NORTH)
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                time.sleep(0.1)
            
            self.finish_guidance()
        else:
            self.get_logger().error("사용자 인식에 실패했습니다. 안내를 종료하고 복귀합니다.")
            self.return_home()

    def finish_guidance(self):
        """
        최종 목적지에 도착했을 때 호출되는 함수입니다.
        """
        self.get_logger().info("도슨트 안내 로봇 목적지 도착")
        
        # 'robot6_goal' 토픽 10초간 발행
        end_time = time.time() + 10
        msg = String()
        msg.data = 'arrived'
        while time.time() < end_time:
            self.goal_publisher.publish(msg)
            time.sleep(0.1)
            
        self.get_logger().info("'robot6_goal' 토픽 발행을 완료했습니다.")
        self.return_home()

    def return_home(self):
        """
        도킹 스테이션으로 복귀합니다.
        """
        self.get_logger().info("복귀 중...")
        home_pose = self.navigator.getPoseStamped(self.home_pose_coords, TurtleBot4Directions.NORTH)
        self.navigator.goToPose(home_pose)
        
        # 복귀는 백그라운드에서 진행될 수 있으므로, 여기서는 바로 종료하지 않고 대기할 수 있습니다.
        # 이 예제에서는 간단히 이동 시작 후 노드를 종료할 수 있도록 처리합니다.
        while not self.navigator.isTaskComplete():
            time.sleep(1)
        
        self.get_logger().info("도킹 스테이션으로 복귀 완료.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    loby_guid_node = LobyGuidNode()
    try:
        rclpy.spin(loby_guid_node)
    except KeyboardInterrupt:
        loby_guid_node.get_logger().info('사용자에 의해 노드가 종료되었습니다.')
    finally:
        loby_guid_node.destroy_node()

if __name__ == '__main__':
    main()
