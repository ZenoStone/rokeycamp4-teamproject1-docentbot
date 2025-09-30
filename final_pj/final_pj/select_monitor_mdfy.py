# 서비스 요청, 확인 다 되면 숫자 입력해서 전시실 안내하라는 서비스 요청

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
# 서비스 정의를 위한 임포트 (예: Trigger.srv 사용)
from std_srvs.srv import Trigger
import time
import threading
import queue
from my_interfaces.srv import GallerySelect  # 사용자 정의 서비스

def input_with_timeout(prompt, timeout):
    q = queue.Queue()
    def input_thread():
        user_input = input(prompt)
        q.put(user_input)
    t = threading.Thread(target=input_thread, daemon=True)
    t.start()
    try:
        return q.get(timeout=timeout)
    except queue.Empty:
        return None

class SelectMonitor(Node):
    def __init__(self):
        super().__init__('select_monitor')
        self.battery_ready = False
        self.robot_guiding = False # 로봇이 전시관 안내 중인지 여부 (이전의 robot_wait 대신 사용)

        # 배터리 상태 확인 서비스 클라이언트 생성
        self.battery_check_cli = self.create_client(Trigger, 'battery_check_service')
        while not self.battery_check_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('배터리 확인 서비스 대기 중...')

        # 로봇 안내 상태 확인 서비스 클라이언트 생성
        # self.robot_state_cli = self.create_client(Trigger, 'robot_state_service')
        # while not self.robot_state_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('로봇 상태 확인 서비스 대기 중...')

        self.gallery_select_cli = self.create_client(GallerySelect, '/robot7/gallery_select_service') ### robot name space check!
        while not self.gallery_select_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('갤러리 선택 서비스 대기 중...')


    def request_battery_status(self):
        request = Trigger.Request()
        future = self.battery_check_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.battery_ready = future.result().success
            self.get_logger().info(f':건전지: 배터리 상태 응답 받음: {self.battery_ready}')
        else:
            self.get_logger().error('배터리 확인 서비스 호출 실패.')
            self.battery_ready = False # 실패 시 안전하게 False로 설정
        return self.battery_ready

    def request_robot_state(self):
        request = Trigger.Request()
        future = self.robot_state_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.robot_guiding = future.result().success
            self.get_logger().info(f':로봇: 로봇 안내 상태 응답 받음: {self.robot_guiding}')
        else:
            self.get_logger().error('로봇 상태 확인 서비스 호출 실패.')
            self.robot_guiding = False # 실패 시 안전하게 False로 설정
        return self.robot_guiding

    def request_gallery_selection(self, number):
        request = GallerySelect.Request()
        request.gallery_number = number
        future = self.gallery_select_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f":사람_달리는_중: 갤러리 {number} 안내 시작: {future.result().message}")
            else:
                self.get_logger().warn(f":경고: 안내 실패: {future.result().message}")
        else:
            self.get_logger().error("갤러리 선택 서비스 응답 실패.")


def main(args=None):
    rclpy.init(args=args)
    node = SelectMonitor()

    node.get_logger().info(":건전지: 배터리 상태 및 로봇 상태 확인 대기 중...")

    try:
        while rclpy.ok():
            # 서비스 호출을 통해 배터리 및 로봇 상태 업데이트
            current_battery_status = node.request_battery_status()
            # current_robot_guiding_status = node.request_robot_state()

            # 두 조건이 모두 만족해야 사용자 입력을 받을 수 있도록 함
            if current_battery_status:# and not current_robot_guiding_status:
                node.get_logger().info(":흰색_확인_표시: 배터리 및 로봇 상태 OK. 전시관 선택 가능.")

                goal_options = [
                    {"name": f"Gallery 1   |  state : {'안내 가능' if not node.robot_guiding else '안내 중'}"},
                    {"name": "Gallery 2   |  state : 안내 불가능"},
                    {"name": "Gallery 3   |  state : 안내 불가능"},
                    {"name": "Gallery 4   |  state : 안내 불가능"},
                    {"name": "종료"} # 5번 옵션 추가
                ]

                options_str = ':둥근_압핀: 방문할 전시관 번호를 입력해주세요:\n'
                for i in range(1, len(goal_options) + 1):
                    options_str += f'    {i}. {goal_options[i - 1]["name"]}\n'
                print(options_str)

                user_input = input_with_timeout(":1234: 번호 입력 (1-5): ", timeout=10)

                if user_input is None:
                    print(":모래시계: 입력 없음, 상태 새로고침...\n")
                    continue
                try:
                    choice = int(user_input)
                    if 1 <= choice <= 5:
                        print(f":흰색_확인_표시: 선택: {goal_options[choice - 1]['name']}")
                        if choice == 5: # 종료 옵션 선택 시
                            print(":손인사: 프로그램 종료.")
                            break
                        elif not node.robot_guiding: # 로봇이 안내 중이 아닐 때만 갤러리 선택 가능
                            node.request_gallery_selection(choice)
                        else:
                            print(":경고: 로봇이 현재 다른 전시관을 안내 중이므로 선택할 수 없습니다. 잠시 후 다시 시도해주세요.")
                    else:
                        print(":경고: 유효하지 않은 선택입니다. 1에서 5 사이의 숫자를 입력해주세요.")
                except ValueError:
                    print(":경고: 유효하지 않은 입력입니다. 숫자를 입력해주세요.")
                print("\n:시계_반대_방향_화살표: 다음 입력 대기 중...\n")
            else:
                if not current_battery_status:
                    node.get_logger().info(":건전지: 배터리 부족. 충전 필요.")
                # if current_robot_guiding_status:
                #     node.get_logger().info(":로봇: 로봇이 현재 전시관 안내 중입니다. 대기 중...")
                print(":모래가_내려오고_있는_모래시계: 시스템 준비 중... (배터리 또는 로봇 상태 확인 중)\n")
                time.sleep(2) # 조건을 만족하지 못하면 2초 대기 후 다시 확인
            rclpy.spin_once(node, timeout_sec=0.1) # 짧은 스핀으로 콜백 처리
    except KeyboardInterrupt:
        print("\n:문: 사용자 중단.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()