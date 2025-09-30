# 배터리 상태 요청받을때, 50퍼 이상이면 성공(true) 보내는 노드.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger # 서비스 정의 임포트

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.battery_percent = 100.0  # 초기값
        self.battery_threshold = 50.0
        # 배터리 상태 구독
        self.subscription = self.create_subscription(
            BatteryState,
            '/robot7/battery_state',
            self.battery_callback,
            10)

        # 배터리 상태 확인 서비스 서버 생성
        self.srv = self.create_service(Trigger, 'battery_check_service', self.battery_check_service_callback)
        self.get_logger().info('배터리 확인 서비스 서버 시작.')

    def battery_callback(self, msg):
        self.battery_percent = msg.percentage * 100.0
        # self.get_logger().info(f':건전지: 배터리 상태 업데이트: {self.battery_percent:.1f}%')

    def battery_check_service_callback(self, request, response):
        # 배터리 잔량이 임계값을 초과하면 True 응답
        if self.battery_percent > self.battery_threshold:
            response.success = True
            response.message = f"배터리 잔량: {self.battery_percent:.1f}% (OK)"
            self.get_logger().info(f'서비스 요청 응답: 배터리 OK ({self.battery_percent:.1f}%)')
        else:
            response.success = False
            response.message = f"배터리 잔량: {self.battery_percent:.1f}% (낮음)"
            self.get_logger().warn(f'서비스 요청 응답: 배터리 낮음 ({self.battery_percent:.1f}%)')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()