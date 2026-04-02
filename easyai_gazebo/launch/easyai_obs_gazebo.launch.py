import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

# this is the function launch  system will look for
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_file = "model.rviz"
    robot_file = "easyai.urdf.xacro"
    package_name = "easyai_gazebo"

    pkg_path = os.path.join(get_package_share_directory(package_name))
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    
    urdf_file = os.path.join(pkg_path, "urdf", robot_file)
    obs_vms_file = os.path.join(pkg_path, "urdf", "obs_vms.urdf.xacro")
    obstacle_file = os.path.join(pkg_path, "urdf", "obstacle.urdf.xacro")

    rviz_config = os.path.join(pkg_path, "rviz", rviz_file)
    
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Robot State Publisher
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    obs_vms_xml = xacro.process_file(obs_vms_file).toxml()
    obst_xml = xacro.process_file(obstacle_file).toxml()

    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    obs_vms_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="obs_vms_state_publisher",
        output='screen',
        parameters=[{'robot_description': obs_vms_xml, 'use_sim_time': use_sim_time}],
        namespace="vms",
    )

    obstacle_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="obstacle_state_publisher",
        output='screen',
        parameters=[{'robot_description': obst_xml, 'use_sim_time': use_sim_time}],
        namespace="obstacle",
    )

    tf_obs_vms = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="vms_tf",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'obs_vms_base_link']  # child 이름 확인!
    )

    tf_obstacle = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="obstacle_tf",
        arguments=['10', '0', '0', '0', '0', '0', 'map', 'obstacle_base_link']  # child 이름 확인!
    )
    

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # spawn_entity = Node(
    #     package='gazebo_ros', 
    #     executable='spawn_entity.py',
    #     output='screen',
    #     arguments=['-topic', 'robot_description', '-entity', 'easyai'],
    # )

    # rviz_start = ExecuteProcess(
    #     cmd=["ros2", "run", "rviz2", "rviz2", "-d", rviz_config], output="screen"
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[
            {'robot_description': doc.toxml()},
            {'obs_vms_description': obs_vms_xml},      # RobotModel #1이 읽을 파라미터
            {'obst_description': obst_xml},    # RobotModel #2가 읽을 파라미터
            {'use_sim_time': use_sim_time},
        ],
    )


    # create and return launch description object
    return LaunchDescription(
        [
            # TimerAction(
            #     period=3.0,
            #     actions=[rviz_start]
            # ),
            #start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
            # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
            #ExecuteProcess(
            #     cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
            #     output="screen",
            # ),
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            robot_state_publisher_node,
            obs_vms_rsp,
            obstacle_rsp,
            tf_obs_vms,
            tf_obstacle,
            joint_state_publisher,
            # tell gazebo to spwan your robot in the world by calling service
            # spawn_entity,
        ]
    )
