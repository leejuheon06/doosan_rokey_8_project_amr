from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Run navigation, yolo_tracker(oakd_viewer), and monitor1 nodes together.

    세 노드 모두 코드 내부에서 '/robot1/...' 형태의 절대 토픽 경로를
    직접 사용하고 있으므로, launch에서 namespace를 따로 지정하지 않습니다.
    namespace를 추가로 주면 '/robot1/robot1/...'처럼 중복 적용되어
    토픽/액션 이름이 어긋나게 됩니다.
    """

    navigation_node = Node(
        package='final_project',
        executable='navigation',
        name='navigation_node',
        output='screen',
    )

    yolo_tracker_node = Node(
        package='final_project',
        executable='yolo_tracker',
        name='detection_node',
        output='screen',
    )

    monitor_node = Node(
        package='final_project',
        executable='monitor',
        name='monitor1_node',
        output='screen',
    )

    alarm_node = Node(
        package='final_project',
        executable='amr_alarm',
        name='alarm_node',
        output='screen',
    )

    return LaunchDescription([
        navigation_node,
        yolo_tracker_node,
        monitor_node,
        alarm_node,
    ])
