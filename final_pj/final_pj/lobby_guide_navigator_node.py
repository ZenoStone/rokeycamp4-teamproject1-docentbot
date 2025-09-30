# 수정중인 노드. select_then_nav_gallery.py에서 카메라 감지 빼고 서비스 받아서 다음스텝 넘어가기까지 완료.
                # 액션 인터페이스 만들어야함. 
# 로비 로봇(/robot6) 네비게이션 노드
# 배터리, 갤러리 선택까지 완료되면, 
# 언도킹,회전, 사람인식, 회전, 출발(wp1까지), 정지, 회전, 사람인식, 회전, 출발, 도착, 대기 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import Twist
import time
import math
import threading

from my_interfaces.srv import GallerySelect
from my_interfaces.action import DetectAndMeasure  # 액션 정의
from rclpy.action import ActionClient
from std_srvs.srv import Trigger # 서비스 정의 임포트


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.navigator = TurtleBot4Navigator()
        self.goal_options = []
        self.home_pose = None

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # 회전을 위한 퍼블리셔
        
        # 사람 확인하는 부분
        self._detect_action_client = ActionClient(self, DetectAndMeasure, 'detect_and_measure')
        # self.get_logger().info("Waiting for 'detect_and_measure' action server...")
        # self.get_logger().info('no goal_msg.111111')
        # self._detect_action_client.wait_for_server()
        # self.get_logger().info('no goal_msg.222222')
        # self.get_logger().info("'detect_and_measure' action server available.")

        # 토픽으로 선택된 경로 인덱스 수신
        self.selected_goal_index = None
        self.executing = False
        self.create_service(GallerySelect, 'gallery_select_service', self.handle_gallery_selection)
        self.get_logger().info("갤러리 선택 서비스 서버 시작")

        self.action_done_event = threading.Event()

    def check_server(self):
        self.get_logger().info("Waiting for 'detect_and_measure' action server...")
        self.get_logger().info('no goal_msg.111211')
        self._detect_action_client.wait_for_server()
        self.get_logger().info('no goal_msg.222222')
        self.get_logger().info("'detect_and_measure' action server available.")

    def send_detect_action_goal(self):
        self.action_done_event.clear()
        goal_msg = DetectAndMeasure.Goal()
        
        self.get_logger().info('Sending goal to action server...')
        self._send_goal_future = self.detect_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # 결과를 기다림
        self.action_done_event.wait() 
        return self.action_result


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.action_result = None
            self.action_done_event.set()
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.action_result = result
        self.action_done_event.set()
        if result and result.success:
            self.get_logger().info(f'[목표 달성] : 최종 측정 거리 {result.final_depth:.2f}m')
        else:
            self.get_logger().warn('[목표 실패] : 최종적으로 거리를 측정하지 못했습니다.')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'[피드백] : 현재 측정 거리 {feedback_msg.feedback.current_depth:.2f}m')

    # 경로 선택 입력받아서 처리하는 서비스 콜백
    def handle_gallery_selection(self, request, response):
        self.get_logger().info("handle_gallery_selection 시작")

        gallery_index = request.gallery_number - 1  # UI는 1부터 시작하므로 -1 보정
        self.get_logger().info(f"{gallery_index + 1}번 갤러리")

        if gallery_index < 0 or gallery_index >= len(self.goal_options):
            response.success = False
            response.message = "유효하지 않은 갤러리 번호입니다."
            return response

        selected_goal = self.goal_options[gallery_index]
        if self.executing:
            response.success = False
            response.message = "로봇이 이미 다른 안내를 수행 중입니다."
            return response

        self.executing = True
        self.get_logger().info(f"갤러리 {gallery_index + 1} 안내 시작.")
        # 별도의 스레드에서 step_through_waypoints 실행
        nav_thread = threading.Thread(target=self.step_through_waypoints, args=(selected_goal['waypoints'], self.home_pose))
        nav_thread.start()

        response.success = True
        response.message = "안내를 시작합니다."
        return response

    # 제자리 회전 함수
    def rotate_in_place(self, angle_degrees, angular_speed=0.5):
        self.get_logger().info("rotate_in_place 시작")

        twist = Twist()
        twist.angular.z = math.copysign(angular_speed, angle_degrees)
        duration = abs(math.radians(angle_degrees)) / angular_speed
        self.navigator.info(f'Rotating {angle_degrees} degrees...')
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1) # rclpy.spin_once 대신 time.sleep 사용
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.navigator.info('Rotation complete.')
    
    # waypoint 진행 함수
    def step_through_waypoints(self, waypoints, home_pose):
        self.get_logger().info("step_through_waypoints 시작")
        
        if self.navigator.getDockedStatus():
            self.get_logger().info("로봇이 도킹된 상태입니다. 도킹 해제 중...")
            self.navigator.undock()

        for i, wp in enumerate(waypoints):
            if not rclpy.ok():
                break

            pose = wp.get('pose', None)
            rotate = wp.get('rotate', None)
            stop = wp.get('stop', False)

            if rotate:
                self.rotate_in_place(rotate)

            if stop:
                self.navigator.info("## Sending DetectAndMeasure goal...")
                result = self.send_detect_action_goal()
                
                if result and result.success:
                    self.get_logger().info("사람 감지 성공! 다음 단계로 진행합니다.")
                else:
                    self.get_logger().warn("사람 감지 실패 또는 타임아웃. 안내를 중단하고 홈으로 복귀합니다.")
                    self.navigator.startToPose(home_pose)
                    # 홈 복귀 완료까지 대기하는 로직 추가 필요
                    self.executing = False
                    return False # 실패 반환

            if pose:
                self.navigator.info(f'Navigating to waypoint {i+1}/{len(waypoints)}')
                self.navigator.startToPose(pose)
                self.navigator.waitUntilNav2Active() # 네비게이션 시작을 기다림
                while not self.navigator.isTaskComplete() and rclpy.ok():
                    time.sleep(0.1)
                if self.navigator.isTaskComplete():
                    self.navigator.info(f'Arrived at waypoint {i+1}')
                else:
                    self.get_logger().warn(f"Waypoint {i+1} 도달 실패.")
                    self.executing = False
                    return False


        self.get_logger().info("모든 웨이포인트를 완료했습니다.")
        self.executing = False
        return True

    def run_navigation_loop(self):
        self.get_logger().info("Ready to receive navigation commands.")
        # MultiThreadedExecutor를 사용하므로, spin()이 모든 콜백을 처리합니다.
        # 별도의 루프는 필요 없습니다.

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    
    node.check_server()

    # 네비게이션 목표 설정
    navigator = node.navigator
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    if not navigator.getDockedStatus():
        navigator.info('Not docked. Docking now...')
        navigator.dock()

    wp1_1 = [-2.9106, -0.0541]
    wp1_goal = [-4.6673, -0.5359]
    wp2_1 = [-3.4332, -0.0504]
    wp2_goal = [-6.0280, -0.2753]
    wp3_1 = [-1.2520, 1.3543]
    wp3_goal = [-2.6589, 1.5903]
    wp4_1 = [-3.4195, 1.3848]
    wp4_goal = [-5.4236, 1.7120]
    home_pose = navigator.getPoseStamped([-0.5457, 0.1060], TurtleBot4Directions.NORTH)
    
    node.home_pose = home_pose
    node.goal_options = [
        {'name': 'Position 1', 'waypoints': [
            {'rotate': -90, 'stop': True}, {'rotate': 90},
            {'pose': navigator.getPoseStamped(wp1_1, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}, {'rotate': 180},
            {'pose': navigator.getPoseStamped(wp1_goal, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}
        ]},
        {'name': 'Position 2', 'waypoints': [
            {'rotate': -90, 'stop': True}, {'rotate': 90},
            {'pose': navigator.getPoseStamped(wp2_1, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}, {'rotate': 180},
            {'pose': navigator.getPoseStamped(wp2_goal, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}
        ]},
        {'name': 'Position 3', 'waypoints': [
            {'rotate': -90, 'stop': True}, {'rotate': 90},
            {'pose': navigator.getPoseStamped(wp3_1, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}, {'rotate': 180},
            {'pose': navigator.getPoseStamped(wp3_goal, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}
        ]},
        {'name': 'Position 4', 'waypoints': [
            {'rotate': -90, 'stop': True}, {'rotate': 90},
            {'pose': navigator.getPoseStamped(wp4_1, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}, {'rotate': 180},
            {'pose': navigator.getPoseStamped(wp4_goal, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}
        ]},
        {'name': 'Exit', 'waypoints': []}
    ]

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
