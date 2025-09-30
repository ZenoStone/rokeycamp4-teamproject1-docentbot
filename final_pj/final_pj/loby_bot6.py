
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time
import math
import threading

from my_interfaces.srv import GallerySelect
from my_interfaces.action import DetectAndMeasure

class LobbyBotNode(Node):
    def __init__(self):
        super().__init__('loby_bot6')
        
        # 1. Navigation and Control Components (from lobby_guide_navigator_node)
        self.navigator = TurtleBot4Navigator()
        self.goal_options = []
        self.home_pose = None
        self.executing = False
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot7/cmd_vel', 10)
        
        # Service to receive gallery selection from UI node
        self.create_service(GallerySelect, 'gallery_select_service', self.handle_gallery_selection)
        self.get_logger().info("갤러리 선택 서비스 서버 시작")

        # Client to check battery status
        # self.battery_check_cli = self.create_client(Trigger, 'battery_check_service')

        # 2. Detection Components (from dd_final)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.last_detected_depth = -1.0  # Depth of the detected person

        # YOLOv8 model loading
        yolo_model_path = '/home/ohjunseok/rokey4_C3_ws/src/final_pj/model/my_best_human.pt'
        try:
            self.model = YOLO(yolo_model_path)
            self.get_logger().info('YOLOv8 모델 로드 성공.')
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
            return

        # Subscribers for camera topics
        self.rgb_subscription = self.create_subscription( 
            Image, '/robot7/oakd/rgb/preview/image_raw', self.rgb_callback, 10)  #### robot6
        self.depth_subscription = self.create_subscription(
            Image, '/robot7/oakd/stereo/image_raw', self.depth_callback, 10) ### robot6

        # Start the background detection thread
        self.detection_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detection_thread.start()

        # 3. Action Server for external requests (as requested)
        self._action_server = ActionServer(
            self,
            DetectAndMeasure,
            'detect_and_measure',
            self.execute_callback)
        self.get_logger().info("사람 감지 액션 서버 시작.")


    # --- Callbacks for Image Processing ---
    def rgb_callback(self, msg):
        with self.lock:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        with self.lock:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # --- Background Detection Logic ---
    def detection_loop(self):
        while rclpy.ok():
            with self.lock:
                rgb_image = self.latest_rgb_image
                depth_image = self.latest_depth_image

            if rgb_image is None or depth_image is None:
                time.sleep(0.1)
                continue

            results = self.model(rgb_image, conf=0.6, verbose=False)
            person_detected = False
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    if self.model.names[int(box.cls[0])] == 'human':
                        person_detected = True
                        b = box.xyxy[0]
                        center_x = int((b[0] + b[2]) / 2)
                        center_y = int((b[1] + b[3]) / 2)

                        # Ensure coordinates are within depth image bounds
                        depth_h, depth_w = depth_image.shape
                        if 0 <= center_y < rgb_image.shape[0] and 0 <= center_x < rgb_image.shape[1]:
                            depth_value = depth_image[min(center_y, depth_h-1), min(center_x, depth_w-1)]
                            if depth_value > 0:
                                with self.lock:
                                    self.last_detected_depth = float(depth_value) / 1000.0
                        break 
                if person_detected:
                    break
            
            if not person_detected:
                with self.lock:
                    self.last_detected_depth = -1.0
            time.sleep(0.1)

    # --- Action Server Execution ---
    def execute_callback(self, goal_handle):
        self.get_logger().info('외부 요청으로 사람 감지 목표 실행...')
        
        success, final_depth = self.wait_for_human_detection(timeout=20.0)

        if success:
            goal_handle.succeed()
            self.get_logger().info(f'목표 성공: {final_depth:.2f}m 에서 사람 감지.')
        else:
            goal_handle.abort()
            self.get_logger().warn('목표 실패: 시간 초과.')
            
        result = DetectAndMeasure.Result()
        result.success = success
        result.final_depth = final_depth
        return result

    # --- Internal Helper Methods ---
    def request_battery_status(self):
        self.get_logger().info("배터리 상태 확인 요청...")
        request = Trigger.Request()
        future = self.battery_check_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"배터리 상태 양호: {future.result().message}")
            return True
        else:
            self.get_logger().error("배터리 상태 불량 또는 서비스 호출 실패.")
            return False

    def wait_for_human_detection(self, timeout=15.0):
        """Internal function to wait for a person, used by the navigation logic."""
        self.get_logger().info(f"{timeout}초 동안 사람을 기다립니다...")
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < timeout:
            with self.lock:
                current_depth = self.last_detected_depth
            
            if 0 < current_depth <= 1.5:
                self.get_logger().info(f"사람 감지 성공! (거리: {current_depth:.2f}m)")
                return True, current_depth

            time.sleep(0.5)
        
        self.get_logger().warn("사람 감지 시간 초과.")
        return False, -1.0

    def rotate_in_place(self, angle_degrees, angular_speed=0.5):
        twist = Twist()
        twist.angular.z = math.copysign(angular_speed, angle_degrees)
        duration = abs(math.radians(angle_degrees)) / angular_speed
        self.get_logger().info(f'{angle_degrees}도 회전 시작...')
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('회전 완료.')

    # --- Main Navigation Logic ---
    def handle_gallery_selection(self, request, response):
        if self.executing:
            response.success = False
            response.message = "로봇이 이미 다른 안내를 수행 중입니다."
            return response

        if not self.request_battery_status():
            response.success = False
            response.message = "배터리가 부족하여 안내를 시작할 수 없습니다."
            return response

        gallery_index = request.gallery_number - 1
        if not (0 <= gallery_index < len(self.goal_options)):
            response.success = False
            response.message = "유효하지 않은 갤러리 번호입니다."
            return response

        selected_goal = self.goal_options[gallery_index]
        self.executing = True
        self.get_logger().info(f"갤러리 {gallery_index + 1} 안내 시작.")
        
        nav_thread = threading.Thread(target=self.step_through_waypoints, args=(selected_goal['waypoints'],))
        nav_thread.start()

        response.success = True
        response.message = "안내를 시작합니다."
        return response

    def step_through_waypoints(self, waypoints):
        self.get_logger().info("경로 안내 시작.")
        
        if self.navigator.getDockedStatus():
            self.get_logger().info("도킹 해제 중...")
            self.navigator.undock()

        # Initial rotation and wait for the first user
        self.rotate_in_place(-90) # 오른쪽으로 90도 회전
        
        # Wait for person
        success, _ = self.wait_for_human_detection()
        if not success:
            self.get_logger().warn("초기 사용자 감지 실패. 홈으로 복귀합니다.")
            self.navigator.startToPose(self.home_pose)
            self.executing = False
            return

        # If person is detected, proceed with the rest of the waypoints
        for i, wp in enumerate(waypoints):
            if not rclpy.ok():
                break

            pose = wp.get('pose', None)
            rotate = wp.get('rotate', None)
            stop_for_person = wp.get('stop', False)

            if rotate:
                self.rotate_in_place(rotate)

            if stop_for_person:
                success, _ = self.wait_for_human_detection()
                if not success:
                    self.get_logger().warn(f"경로 {i+1}에서 사용자 감지 실패. 홈으로 복귀합니다.")
                    self.navigator.startToPose(self.home_pose)
                    self.executing = False
                    return

            if pose:
                self.get_logger().info(f'경로 {i+1}/{len(waypoints)}로 이동 중...')
                self.navigator.startToPose(pose)
                while not self.navigator.isTaskComplete() and rclpy.ok():
                    time.sleep(0.1)
                
                if self.navigator.isTaskComplete():
                    self.get_logger().info(f'경로 {i+1} 도착.')
                else:
                    self.get_logger().warn(f"경로 {i+1} 도달 실패.")
                    self.executing = False
                    return

        self.get_logger().info("모든 경로 안내 완료. 홈으로 복귀합니다.")
        self.navigator.startToPose(self.home_pose)
        self.executing = False


def main(args=None):
    rclpy.init(args=args)
    node = LobbyBotNode()
    
    # Setup initial pose and waypoints
    navigator = node.navigator
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    if not navigator.getDockedStatus():
        node.get_logger().info('도킹되지 않음. 도킹 시작...')
        navigator.dock()

    # Waypoint Definitions
    wp1_1 = [-2.9106, -0.0541]
    wp1_goal = [-4.6673, -0.5359]
    wp2_1 = [-3.4332, -0.0504]
    wp2_goal = [-6.0280, -0.2753]
    # ... (add other waypoints as needed)

    node.home_pose = navigator.getPoseStamped([-0.5457, 0.1060], TurtleBot4Directions.NORTH)
    
    node.goal_options = [
        {'name': 'Gallery 1', 'waypoints': [
            # Initial rotation and detection is handled before this loop
            {'rotate': 90}, # Turn back to original orientation
            {'pose': navigator.getPoseStamped(wp1_1, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True},
            {'rotate': 180},
            {'pose': navigator.getPoseStamped(wp1_goal, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}
        ]},
        {'name': 'Gallery 2', 'waypoints': [
            {'rotate': 90},
            {'pose': navigator.getPoseStamped(wp2_1, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True},
            {'rotate': 180},
            {'pose': navigator.getPoseStamped(wp2_goal, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True}
        ]},
        # ... (add other galleries)
    ]

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('사용자 요청으로 종료.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
