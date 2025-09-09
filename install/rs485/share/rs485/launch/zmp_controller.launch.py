from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    my_launch = LaunchDescription()
    #디스크립션을 선언한다음 add_actiong으로 추가해주는 방식의 런치구성

    #변수 = Node(패키지, 실행파일, 아웃풋, 변수, 네임스페이스, 노드...)

    imu_node = Node(
        package='rs485',
        executable='imu_pub',
        output='screen',
    )

    zmp_controller_node = Node(
        package='rs485',
        executable='zmp_test_imu',
        output='screen',
    )
    diff_cmdvel_node = Node(
        package='rs485',
        executable='diff_cmd_vel',
        output='screen',
    )

    my_launch.add_action(imu_node)
    my_launch.add_action(zmp_controller_node)
    my_launch.add_action(diff_cmdvel_node )
    return my_launch
