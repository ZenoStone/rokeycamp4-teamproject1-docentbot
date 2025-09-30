import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time
import sys
import select

from my_interfaces.srv import GallerySelect

def get_input_with_timeout(prompt, timeout):
    """Waits for user input for a specified duration without creating extra threads."""
    sys.stdout.write(prompt)
    sys.stdout.flush()
    
    # select.select monitors file descriptors. Here, we monitor stdin.
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        # If stdin is ready, read the line.
        return sys.stdin.readline().strip()
    else:
        # If timeout occurs, return None.
        return None

class SelectMonitor(Node):
    def __init__(self):
        super().__init__('select_monitor_modified')
        self.robot_guiding = False  # Simplified state: assume robot is available initially

        # --- Service Clients ---
        self.battery_check_cli = self.create_client(Trigger, 'battery_check_service')
        self.gallery_select_cli = self.create_client(GallerySelect, '/robot6/gallery_select_service') ### robot
        
        self.get_logger().info("서비스 클라이언트 초기화 중...")
        if not self.battery_check_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('배터리 확인 서비스를 찾을 수 없습니다. 노드를 종료합니다.')
            sys.exit(1)
        if not self.gallery_select_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('갤러리 선택 서비스를 찾을 수 없습니다. 노드를 종료합니다.')
            sys.exit(1)
        self.get_logger().info("모든 서비스가 연결되었습니다.")

    def call_service(self, client, request):
        """Generic async service call helper."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) # 5초 타임아웃
        if future.done():
            return future.result()
        self.get_logger().warn(f'{client.srv_name} 서비스 호출 시간 초과.')
        return None

    def run_main_loop(self):
        """The main operational loop of the node."""
        while rclpy.ok():
            # 1. Check Battery Status
            battery_req = Trigger.Request()
            battery_res = self.call_service(self.battery_check_cli, battery_req)

            if not battery_res or not battery_res.success:
                self.get_logger().warn(f"배터리 상태 확인 실패 또는 부족: {battery_res.message if battery_res else '타임아웃'}. 2초 후 재시도...")
                time.sleep(2)
                continue

            # 2. Display Options and Get User Input
            self.get_logger().info("배터리 상태 양호. 전시관 선택 가능.")
            goal_options = [
                f"Gallery 1   |  state : {'안내 가능' if not self.robot_guiding else '안내 중'}",
                "Gallery 2   |  state : 안내 불가능",
                "Gallery 3   |  state : 안내 불가능",
                "Gallery 4   |  state : 안내 불가능",
                "종료"
            ]
            options_str = '\n:둥근_압핀: 방문할 전시관 번호를 입력해주세요:\n' + \
                          '\n'.join([f'    {i+1}. {name}' for i, name in enumerate(goal_options)])
            print(options_str)

            user_input = get_input_with_timeout("\n:1234: 번호 입력 (1-5): ", timeout=15.0)

            if user_input is None:
                print("\n:모래시계: 입력 시간이 초과되었습니다. 상태를 새로고침합니다.\n")
                continue

            # 3. Process User Input
            try:
                choice = int(user_input)
                if not (1 <= choice <= len(goal_options)):
                    print("\n:경고: 유효하지 않은 선택입니다. 1에서 5 사이의 숫자를 입력해주세요.\n")
                    continue
                
                print(f":흰색_확인_표시: 선택: {goal_options[choice - 1]}")

                if choice == 5: # Exit option
                    print(":손인사: 프로그램을 종료합니다.")
                    break
                
                if self.robot_guiding:
                    print("\n:경고: 로봇이 현재 다른 안내를 수행 중입니다. 잠시 후 다시 시도해주세요.\n")
                    continue

                # 4. Call Gallery Selection Service
                gallery_req = GallerySelect.Request()
                gallery_req.gallery_number = choice
                gallery_res = self.call_service(self.gallery_select_cli, gallery_req)

                if gallery_res:
                    if gallery_res.success:
                        self.get_logger().info(f":사람_달리는_중: 안내 시작 요청 성공: {gallery_res.message}")
                        self.robot_guiding = True # Assume robot is now busy
                    else:
                        self.get_logger().warn(f":경고: 안내 시작 요청 실패: {gallery_res.message}")
                else:
                    self.get_logger().error("갤러리 선택 서비스 응답을 받지 못했습니다.")
                
                # After a request, you might want to wait or reset the state
                # For now, we just loop. A real implementation might need a state update mechanism.
                self.robot_guiding = False # Simplified for this example

            except ValueError:
                print("\n:경고: 유효하지 않은 입력입니다. 숫자를 입력해주세요.\n")
            except Exception as e:
                self.get_logger().error(f"오류 발생: {e}")

            print("\n:시계_반대_방향_화살표: 다음 입력을 기다립니다...\n")
            time.sleep(1) # Short delay before next loop

def main(args=None):
    rclpy.init(args=args)
    node = SelectMonitor()
    try:
        node.run_main_loop()
    except KeyboardInterrupt:
        print("\n:문: 사용자 요청으로 프로그램 중단.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
