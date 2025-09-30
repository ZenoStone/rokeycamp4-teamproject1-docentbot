import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from my_interfaces.action import DetectAndMeasure
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time
import threading

class DetectAndMeasureActionServer(Node):
    def __init__(self):
        super().__init__('detect_and_measure_action_server2')
        self._action_server = ActionServer(
            self,
            DetectAndMeasure,
            'detect_and_measure',
            self.execute_callback)

        self.rgb_subscription = self.create_subscription(
            Image, '/robot7/oakd/rgb/preview/image_raw', self.rgb_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/robot7/oakd/stereo/image_raw', self.depth_callback, 10)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.last_depth = -1.0
        self.goal_reached = threading.Event()

        # Load YOLOv8 model
        yolo_model_path = '/home/ohjunseok/rokey4_C3_ws/src/DepthToEstimateDistance/model/my_best_human.pt'
        self.model = YOLO(yolo_model_path)
        self.get_logger().info('YOLOv8 model loaded successfully.')

        self.detection_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detection_thread.start()

    def rgb_callback(self, msg):
        with self.lock:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        with self.lock:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def detection_loop(self):
        while rclpy.ok():
            with self.lock:
                rgb_image = self.latest_rgb_image
                depth_image = self.latest_depth_image

            if rgb_image is None or depth_image is None:
                time.sleep(0.1)
                self.get_logger().info('rgb_image is None or depth_image is None.')

                continue

            results = self.model(rgb_image, conf=0.6, verbose=False)
            person_detected = False
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    self.get_logger().info(f'### {class_name}.')
                    if class_name == 'human':
                        person_detected = True
                        b = box.xyxy[0]
                        xmin, ymin, xmax, ymax = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                        center_x = int((xmin + xmax) / 2)
                        center_y = int((ymin + ymax) / 2)

                        # Scale coordinates from preview to full resolution
                        preview_height, preview_width, _ = rgb_image.shape
                        depth_height, depth_width = depth_image.shape
                        
                        scale_x = depth_width / preview_width
                        scale_y = depth_height / preview_height
                        
                        depth_x = int(center_x * scale_x)
                        depth_y = int(center_y * scale_y)
                        self.get_logger().info(f'### {depth_x}, {depth_y}.')

                        if center_x < rgb_image.shape[1] and center_y < rgb_image.shape[0] and center_y < rgb_image.shape[0] and center_x < rgb_image.shape[1]:
                            depth_value = depth_image[depth_y, depth_x]
                            if depth_value > 0:
                                depth_in_meters = float(depth_value) / 1000.0
                                with self.lock:
                                    self.last_depth = depth_in_meters
                                
                                if 0 < depth_in_meters <= 1.5:
                                    self.goal_reached.set()
                        break
                if person_detected:
                    break
            
            if not person_detected:
                with self.lock:
                    self.last_depth = 333.0
                self.goal_reached.clear()

            # time.sleep(0.2)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = DetectAndMeasure.Feedback()
        self.goal_reached.clear()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                return DetectAndMeasure.Result()

            with self.lock:
                current_depth = self.last_depth
            
            feedback_msg.current_depth = current_depth
            goal_handle.publish_feedback(feedback_msg)

            if self.goal_reached.is_set():
                goal_handle.succeed()
                result = DetectAndMeasure.Result()
                result.success = True
                result.final_depth = current_depth
                self.get_logger().info(f'Goal succeeded: Person within 1.5m at {current_depth:.2f}m.')
                return result

            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    action_server = DetectAndMeasureActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    try:
        executor.spin()
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
