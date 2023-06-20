import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    gazebo_world_path = os.path.join(get_package_share_directory('car_sim_gazebo'), 'worlds', 'jarbay.world')
    gazebo_options_dict = dict(
        world = gazebo_world_path
    )

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=gazebo_options_dict.items()
    )

    car_sim_options = dict(
        start_x = '0',
        start_y = '0',
        start_z = '0',
        start_yaw = '0',
        pub_tf = 'true',
        tf_freq = '100.0',
        blue = 'false'
    )

    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('car_sim_gazebo'), 'launch', 'car_sim_robot.launch.py')
        ]),
        launch_arguments=car_sim_options.items()
    )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='single_vehicle_viz',
    #     arguments=['-d', os.path.join(get_package_share_directory('car_sim_gazebo'), 'rviz', 'single_vehicle_example.rviz')]
    # )

    return LaunchDescription([
        gazebo_simulator,
        spawn_car,
        # rviz
    ])
