## detack_depth.py 사람 감지, 뎁스 출력 노드

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import tf2_ros
from geometry_msgs.msg import PointStamped
import rclpy.time

class ObjectDepthEstimator(Node):
    def __init__(self):
        super().__init__('detack_depth')

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.K = None

        self.rgb_sub = self.create_subscription(Image, '/robot6/oakd/rgb/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/robot6/oakd/stereo/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/robot6/oakd/rgb/camera_info', self.camera_info_callback, 10)

        self.model = YOLO('/home/rokey/rokey4_C3_ws/src/my_package/my_best_human.pt')  # YOLOv8 학습 모델 경로

        self.timer = self.create_timer(0.5, self.process)

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        fx, fy, cx, cy = self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]
        self.get_logger().info(f'Camera intrinsics loaded: fx={fx}, fy={fy}, cx={cx}, cy={cy}')

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def process(self):
        if self.rgb_image is None or self.depth_image is None or self.K is None:
            return

        image = self.rgb_image.copy()
        depth = self.depth_image.copy()
        results = self.model(image, verbose=False)[0]

        for box in results.boxes:
            cls = int(box.cls[0])
            label = self.model.names[cls]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            u = (x1 + x2) // 2
            v = (y1 + y2) // 2

            # 5x5 주변 픽셀 평균 거리 계산
            patch = depth[max(0, v-2):v+3, max(0, u-2):u+3]
            valid_patch = patch[(patch > 200) & (patch < 5000)]  # 0.2m ~ 5m
            if valid_patch.size == 0:
                self.get_logger().warn(f"No valid depth at center of {label}")
                continue

            z = np.mean(valid_patch) / 1000.0  # mm → m

            # 거리 출력
            self.get_logger().info(f"[{label}] at ({u},{v}): approx distance = {z:.2f} m")

            # 시각화
            cv2.rectangle(image, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.circle(image, (u, v), 4, (0, 255, 255), -1)
            cv2.putText(image, f"{label} {z:.2f}m", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

        # 이미지 창 출력
        cv2.imshow("YOLO Detection with Depth", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDepthEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()