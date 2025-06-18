import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    """
    Loads the robot description from a Xacro file, using parameters from a YAML file.

    @param robot_description_path: Path to the robot's Xacro file.
    @param vehicle_params_path: Path to the YAML file containing the vehicle parameters.
    @return: A string containing the robot's URDF XML description.
    """
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()})

    return robot_description.toxml()


def start_vehicle_control():
    """
    Starts the necessary controllers for the vehicle's operation in ROS 2.

    @return: A tuple containing ExecuteProcess actions for the joint state, forward velocity, 
             and forward position controllers.
    """
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller)


def generate_launch_description():
    # --- 1. 定义核心启动参数 ---
    package_name = "gazebo_ackermann_steering_vehicle"
    package_path = get_package_share_directory(package_name)
    world_path = os.path.join(package_path, 'worlds', 'cafe2.world')

    # !! 新增：统一管理 use_sim_time 参数 !!
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/recorded time when true')

    world_arg = DeclareLaunchArgument(
        'world', 
        default_value=world_path,
        description='Specify the world file for Gazebo')
    
    # 定义初始位姿参数
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Initial X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.1', description='Initial Z position')
    roll_arg = DeclareLaunchArgument('R', default_value='0.0', description='Initial Roll')
    pitch_arg = DeclareLaunchArgument('P', default_value='0.0', description='Initial Pitch')
    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0', description='Initial Yaw')

    # --- 2. 获取启动配置 ---
    use_sim_time = LaunchConfiguration('use_sim_time') # 读取 use_sim_time 的值
    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # --- 3. 设置文件路径和加载机器人描述 ---
    robot_description_path = os.path.join(package_path, 'model', 'vehicle.xacro')
    gz_bridge_params_path = os.path.join(package_path, 'config', 'ros_gz_bridge.yaml')
    vehicle_params_path = os.path.join(package_path, 'config', 'parameters.yaml')
    robot_description = load_robot_description(robot_description_path, vehicle_params_path)

    # --- 4. 定义和配置节点 ---
    
    # 启动 Gazebo 仿真
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file],
                          'on_exit_shutdown': 'true'}.items())

    # 在 Gazebo 中生成机器人模型
    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', "ackermann_steering_vehicle",
                   '-string', robot_description,
                   '-x', x, '-y', y, '-z', z,
                   '-R', roll, '-P', pitch, '-Y', yaw,
                   '-allow_renaming', 'false'],
        output='screen')

    # 机器人状态发布节点 (robot_state_publisher)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time  # !! 修改点：使用统一的启动参数 !!
        }])

    # ROS-Gazebo 桥接节点
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        parameters=[{'use_sim_time': use_sim_time}], # !! 新增：为桥接节点添加 use_sim_time !!
        output='screen')
        
    # 车辆控制器节点
    vehicle_controller_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='vehicle_controller',
        parameters=[
            vehicle_params_path,
            {'use_sim_time': use_sim_time} # !! 新增：为你的控制器节点添加 use_sim_time !!
        ],
        output='screen')
     # Load vehicle controller node
    vehicle_controller_node = Node(package='gazebo_ackermann_steering_vehicle',
                                   executable='vehicle_controller',
                                   parameters=[vehicle_params_path],
                                   output='screen')
    # #####################################

    # 增加 cmd_vel 到车辆控制信号的转换节点
    cmd_vel_to_vehicle_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='cmd_vel_to_vehicle',  # 你需要创建这个可执行文件
        name='cmd_vel_to_vehicle_converter',
        parameters=[vehicle_params_path],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),  # 接收来自 nav2 的 cmd_vel
            ('/steering_angle', '/steering_angle'),  # 发布转向角度
            ('/velocity', '/velocity')  # 发布速度
        ],
        output='screen'
    )
  
    # #####################################

    # --- 5. 加载 ros2_control 控制器 ---
    joint_state, forward_velocity, forward_position = start_vehicle_control()

    # --- 6. 构建最终的 LaunchDescription ---
    return LaunchDescription([
        # 注册事件处理器，确保在模型生成后加载控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_model_gazebo_node,
                on_exit=[joint_state]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state,
                on_exit=[forward_velocity, forward_position]
            )
        ),

        # 添加所有启动参数
        use_sim_time_arg,
        world_arg,
        x_arg, y_arg, z_arg,
        roll_arg, pitch_arg, yaw_arg,

        # 添加所有节点和启动项
        gazebo_launch,
        spawn_model_gazebo_node,
        robot_state_publisher_node,
        vehicle_controller_node,
        cmd_vel_to_vehicle_node,
        gz_bridge_node
    ])