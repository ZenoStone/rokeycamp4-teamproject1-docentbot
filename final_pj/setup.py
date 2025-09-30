from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'final_pj'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        # (os.path.join('share', package_name, 'action'), glob('action/*.action')),

    ],
    install_requires=['setuptools', 'turtlebot4_navigation'], ### turtlebot4_navigation 추가함
    zip_safe=True,
    maintainer='ohjunseok',
    maintainer_email='ohjunseok@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            # 옛날꺼. (토픽)
            'gallery_guide_navigator_node = final_pj.gallery_guide_navigator_node:main',
            'human_presence_detector_node = final_pj.human_presence_detector_node:main',
            'visitor_distance_estimator_pub = final_pj.visitor_distance_estimator_pub:main',

            # test
            'external_battery_check_request = final_pj.external_battery_check_request:main',    # 배터리 상태를 보내야 하는지 아닌지. 이제 필요없는거.
            'srv_server_test = final_pj.srv_server_test:main',  # 대충 테스트용. 필요x
            
            # 여기부터.
            'battery_monitor_node6 = final_pj.battery_monitor_node6:main',    # 배터리 상태로 안내 가능인지 아닌지
            'battery_monitor_node7 = final_pj.battery_monitor_node7:main',    # 배터리 상태로 안내 가능인지 아닌지

            
            'select_monitor = final_pj.select_monitor:main',    # 아래꺼랑 비슷한데 덜손댄거.
            'select_monitor_mdfy = final_pj.select_monitor_mdfy:main',            # 안내 가능한 상태라면, 어디를 안내할건지 선택받기
            
            'select_then_nav_gallery = final_pj.select_then_nav_gallery:main',  # 선택 서버 겸 네비 노드. (카메라 감지 빼고 서비스 받아서 다음스텝 넘어가기까지 완료.)
            
            
            'robot_state_server = final_pj.robot_state_server:main',    # 현상태가 안내 중인지 아닌지       # 아직 손댈부분 많음.
            'dummy_detect_server = final_pj.dummy_detect_server:main',# action server 테스트용 더미    # 현상태가 안내 중인지 아닌지       # 아직 손댈부분 많음.
            

            'dd_final = final_pj.dd_final:main',# action server
            'dd_client = final_pj.detect_and_measure_action_client:main', #### 사용자 인식 action client

            'lobby_guide_navigator_node = final_pj.lobby_guide_navigator_node:main',

            'loby6 = final_pj.loby_bot6:main', ##### Gemini로 수정한 로비 코드
            'smm_mdfy = final_pj.smm_mdfy:main', ##### Gemini로 수정한 갤러리 선택 코드

            # 아르코마커 노드
            'aruco_detect_response = final_pj.aruco_detect_response:main',


            # mqtt 통신용 로봇 통신용. 초안(gpt) 아마 안될듯.
            'robot_state_publisher = final_pj.mqtt.robot_state_publisher:main',
            'robot_state_subscriber = final_pj.mqtt.robot_state_subscriber:main',


            'robot7_status_publisher = final_pj.mqtt.robot7_status_publisher:main',
            # ' = final_pj.:main',
            
        ],
    },
)
