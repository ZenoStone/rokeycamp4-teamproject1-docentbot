# madic_waypoint.py 
# 노드이름 : waypoint_navigator
# 토픽 : human_check (Bool)
# 콜백 : human_check_callback (사람 감지 여부 수신)
# 기능 : 목적지 숫자 입력 -> 경로 따라 웨이포인트 네비게이션 수행 -> 중간중간 대기 (사람 감지)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import time

# ================================
# Waypoint 좌표 변수 선언
# ================================
wp1 = (-1.278993, -0.200784)
wp2 = (-1.330508, -0.593572)
wp3 = (-1.874826, -0.604343)
wp4 = (-2.024864, -2.160810)
wp5 = (-0.761671, -0.852567)
wp6 = (0.0343325, -1.96793)

# ================================
# 목표 위치 리스트 정의 (전역 상수)
# ================================
GOAL_OPTIONS = [
    {
        'name': 'Position 1',
        'waypoints': [
            {'pos': wp1, 'dir': TurtleBot4Directions.WEST, 'stop': True},
            {'pos': wp1, 'dir': TurtleBot4Directions.EAST, 'stop': False},
            {'pos': wp1, 'dir': TurtleBot4Directions.WEST, 'stop': True},
            {'pos': wp1, 'dir': TurtleBot4Directions.EAST, 'stop': False},
            {'pos': wp2, 'dir': TurtleBot4Directions.SOUTH, 'stop': False},
            {'pos': wp3, 'dir': TurtleBot4Directions.EAST, 'stop': False},
            {'pos': wp3, 'dir': TurtleBot4Directions.WEST, 'stop': True},
            {'pos': wp3, 'dir': TurtleBot4Directions.EAST, 'stop': False},
            {'pos': wp4, 'dir': TurtleBot4Directions.EAST, 'stop': False},
            {'pos': wp4, 'dir': TurtleBot4Directions.WEST, 'stop': True},
        ]
    },
    {
        'name': 'Position 2',
        'waypoints': [
            {'pos': wp5, 'dir': TurtleBot4Directions.SOUTH, 'stop': True}
        ]
    },
    {
        'name': 'Position 3',
        'waypoints': [
            {'pos': wp6, 'dir': TurtleBot4Directions.EAST, 'stop': True}
        ]
    },
    {
        'name': 'Exit',
        'waypoints': []
    }
]

# ================================
# Waypoint Navigator Node 정의
# ================================
class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.navigator = TurtleBot4Navigator()
        self.human_detected = False
        self._last_msg_time = time.time()

        self.create_subscription(Bool, 'human_check', self.human_check_callback, 10)

    def human_check_callback(self, msg):
        self.human_detected = msg.data
        self._last_msg_time = time.time()

    def step_through_waypoints(self, waypoints, home_pose):
        stop_indices = [i for i, wp in enumerate(waypoints) if wp.get('stop', False)]
        last_stop_index = stop_indices[-1] if stop_indices else -1

        for i, wp in enumerate(waypoints):
            pose = wp['pose']
            stop = wp.get('stop', False)
            is_last_stop = (i == last_stop_index)

            self.navigator.info(f'Navigating to waypoint {i+1}/{len(waypoints)}')
            self.navigator.startToPose(pose)

            timeout = 60
            start_time = time.time()
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.2)
                if time.time() - start_time > timeout:
                    self.navigator.error(f"Timeout while navigating to waypoint {i+1}")
                    break

            self.navigator.info(f'Arrived at waypoint {i+1}')

            if stop:
                self.human_detected = False
                self.navigator.info("Waiting for human_check to be True...")

                while not self.human_detected and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.2)
                    if time.time() - start_time > timeout:
                        self.navigator.error(f"Timeout waiting for human_check at waypoint {i+1}")
                        break

                self.navigator.info("human_check received. Proceeding...")

                if is_last_stop:
                    self.navigator.info("Monitoring human presence to return Home after disappearance...")
                    start_time = None

                    while rclpy.ok():
                        rclpy.spin_once(self, timeout_sec=0.5)
                        if not self.human_detected:
                            if start_time is None:
                                start_time = time.time()
                                self.navigator.info("human_check lost. Starting return timer...")
                            elif time.time() - start_time >= 10.0:
                                self.navigator.info("Human disappeared. Returning Home...")
                                self.navigator.startToPose(home_pose)
                                while not self.navigator.isTaskComplete():
                                    rclpy.spin_once(self)
                                self.navigator.info("Docking...")
                                self.navigator.dock()
                                return
                        else:
                            start_time = None

# ================================
# Main 함수
# ================================
def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    navigator = node.navigator

    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    navigator.undock()

    navigator.info('Welcome to the mail delivery service.')

    home_pose = navigator.getPoseStamped([-0.6, 0.0], TurtleBot4Directions.NORTH)

    while rclpy.ok():
        options_str = 'Please enter the number corresponding to the desired robot goal position:\n'
        for i, option in enumerate(GOAL_OPTIONS):
            options_str += f'    {i}. {option["name"]}\n'

        raw_input = input(f'{options_str}Selection: ')

        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'Invalid selection: {raw_input}')
            continue

        if selected_index < 0 or selected_index >= len(GOAL_OPTIONS):
            navigator.error(f'Selection out of range: {selected_index}')
            continue

        selected_goal = GOAL_OPTIONS[selected_index]

        if selected_goal['name'] == 'Exit':
            navigator.info('Going Home...')
            navigator.startToPose(home_pose)
            navigator.info('Docking before exiting...')
            navigator.dock()
            break

        else:
            navigator.info(f'Navigating step-by-step to {selected_goal["name"]}')
            waypoints = [
                {
                    'pose': navigator.getPoseStamped(wp['pos'], wp['dir']),
                    'stop': wp['stop']
                } for wp in selected_goal['waypoints']
            ]
            node.step_through_waypoints(waypoints, home_pose)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
