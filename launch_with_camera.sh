#!/bin/bash

# 启动带摄像头可视化的Gazebo车辆仿真
# Start Gazebo vehicle simulation with camera visualization

echo "🚗 Starting Ackermann steering vehicle simulation with camera visualization..."

# 设置工作空间
WORKSPACE_DIR="/media/onion/data/sample/gazebo_car"
cd $WORKSPACE_DIR

# 设置ROS2环境
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "⚡ 启动仿真..."
echo "⚡ Starting simulation..."

# 在后台启动车辆仿真
ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py use_sim_time:=true &

GAZEBO_PID=$!

# 等待Gazebo完全启动
echo "⏳ 等待Gazebo启动完成..."
echo "⏳ Waiting for Gazebo to start..."
sleep 10

echo "📹 Starting RViz2 camera visualization..."
rviz2 -d $WORKSPACE_DIR/src/gazebo_ackermann_steering_vehicle/config/camera_view.rviz use_sim_time:=true & 
RVIZ_PID=$!

echo "✅ 系统启动完成！"
echo "✅ System startup completed!"
echo ""
echo "   🔄 Available topics:"
echo "      - /camera/image_raw (摄像头图像)"
echo "      - /camera/info (摄像头信息)"
echo "      - /steering_angle (转向角度)"
echo "      - /velocity (速度)"
echo ""
echo "❌ Press Ctrl+C to stop all processes"

# 启动SLAM工具箱
# echo "🚀 启动SLAM工具箱..."
# echo "🚀 Starting SLAM Toolbox..."

# # 启动SLAM工具箱，加载预设的SLAM配置文件
# ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/media/onion/data/sample/gazebo_car/src/gazebo_ackermann_steering_vehicle/config/slam.yaml use_sim_time:=true
# SLAM_PID=$!

# 启动导航堆栈
# ros2 launch nav2_bringup navigation_launch.py  use_sim_time:=true   map:=/media/onion/data/sample/gazebo_car/src/gazebo_ackermann_steering_vehicle/map/slam_map.yaml   params_file:=/media/onion/data/sample/gazebo_car/src/gazebo_ackermann_steering_vehicle/config/nav2_params.yaml



# 等待用户中断
wait $GAZEBO_PID $RVIZ_PID

