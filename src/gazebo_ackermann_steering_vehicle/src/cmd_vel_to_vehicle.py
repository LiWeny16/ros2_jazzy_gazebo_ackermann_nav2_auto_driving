#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class Nav2AckermannConverter(Node):
    def __init__(self):
        super().__init__('nav2_ackermann_converter')
        
        # 声明参数 - 优化为与Nav2 MPPI/SMAC配合使用
        self.declare_parameter('wheelbase', 0.3)  # 直接使用轴距
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('max_steering_angle', 0.6108652)
        self.declare_parameter('max_velocity', 0.5)  # 与nav2_params.yaml中vx_max一致
        self.declare_parameter('min_turning_radius', 0.2)  # 与MPPI配置一致
        self.declare_parameter('velocity_gain', 1.0)  # 速度增益
        self.declare_parameter('steering_gain', 1.0)  # 转向增益
        self.declare_parameter('min_velocity_threshold', 0.001)  # 与nav2配置一致
        self.declare_parameter('angular_velocity_gain', 1.0)  # 角速度增益
        self.declare_parameter('use_enhanced_conversion', True)  # 使用增强转换算法
        
        # 获取参数
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.min_turning_radius = self.get_parameter('min_turning_radius').get_parameter_value().double_value
        self.velocity_gain = self.get_parameter('velocity_gain').get_parameter_value().double_value
        self.steering_gain = self.get_parameter('steering_gain').get_parameter_value().double_value
        self.min_velocity_threshold = self.get_parameter('min_velocity_threshold').get_parameter_value().double_value
        self.angular_velocity_gain = self.get_parameter('angular_velocity_gain').get_parameter_value().double_value
        self.use_enhanced_conversion = self.get_parameter('use_enhanced_conversion').get_parameter_value().bool_value
        
        # 计算物理限制
        self.max_angular_velocity = self.max_velocity / self.min_turning_radius  # 最大角速度
        self.min_steering_radius = self.wheelbase / math.tan(self.max_steering_angle)  # 最小转弯半径
        
        # Publishers - 发布到您现有的控制接口
        self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)
        self.velocity_pub = self.create_publisher(Float64, '/velocity', 10)
        
        # Subscriber - 订阅Nav2控制器输出
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel',  # Nav2 MPPI控制器的输出
            self.enhanced_cmd_vel_callback, 
            10
        )
        
        # 状态变量用于平滑控制
        self.previous_steering = 0.0
        self.previous_velocity = 0.0
        self.steering_filter_alpha = 0.3  # 转向平滑系数
        self.velocity_filter_alpha = 0.5  # 速度平滑系数
        
        self.get_logger().info('Nav2 Ackermann Converter initialized!')
        self.get_logger().info(f'Wheelbase: {self.wheelbase:.3f} m')
        self.get_logger().info(f'Max steering angle: {math.degrees(self.max_steering_angle):.1f}°')
        self.get_logger().info(f'Min turning radius: {self.min_turning_radius:.3f} m')
        self.get_logger().info(f'Max angular velocity: {self.max_angular_velocity:.3f} rad/s')
        self.get_logger().info(f'Enhanced conversion: {self.use_enhanced_conversion}')

    def enhanced_cmd_vel_callback(self, msg: Twist):
        """增强的twist到ackermann转换 - 专为Nav2 MPPI优化"""
        linear_vel = msg.linear.x * self.velocity_gain
        angular_vel = msg.angular.z * self.angular_velocity_gain
        
        # 处理停止状态
        if abs(linear_vel) < self.min_velocity_threshold and abs(angular_vel) < 0.001:
            self.publish_control_commands(0.0, 0.0)
            return
        
        # 使用增强转换算法
        if self.use_enhanced_conversion:
            steering_angle, final_velocity = self.enhanced_ackermann_conversion(linear_vel, angular_vel)
        else:
            steering_angle, final_velocity = self.basic_ackermann_conversion(linear_vel, angular_vel)
        
        # 应用平滑滤波
        steering_angle = self.apply_steering_filter(steering_angle)
        final_velocity = self.apply_velocity_filter(final_velocity)
        
        # 发布控制命令
        self.publish_control_commands(steering_angle, final_velocity)
        
        # 调试输出
        if abs(final_velocity) > 0.01 or abs(steering_angle) > 0.01:
            self.get_logger().info(
                f'Nav2->Ackermann: v_in={linear_vel:.3f} ω_in={angular_vel:.3f} -> '
                f'v_out={final_velocity:.3f} δ={math.degrees(steering_angle):.1f}°',
                throttle_duration_sec=0.5
            )

    def enhanced_ackermann_conversion(self, linear_vel, angular_vel):
        """增强的阿克曼转换算法 - 专门优化掉头和复杂机动"""
        
        # 处理纯转向情况 - 简化原地转向逻辑
        if abs(linear_vel) < self.min_velocity_threshold:
            if abs(angular_vel) > 0.05:  # 原地转向
                # 根据角速度大小决定转向角度和速度
                steering_angle = math.copysign(self.max_steering_angle * 0.9, angular_vel)
                
                # 原地转向：使用低速前进或后退辅助转向
                if abs(angular_vel) > 1.5:  # 大角速度时
                    # 交替前进后退以实现更有效的掉头
                    velocity = math.copysign(0.2, angular_vel)
                else:
                    velocity = math.copysign(0.15, angular_vel)
                    
                return steering_angle, velocity
            else:
                return 0.0, 0.0
        
        # 标准阿克曼转换
        if abs(angular_vel) < 0.001:
            # 直线运动
            return 0.0, linear_vel
        
        # 计算转弯半径
        turning_radius = abs(linear_vel / angular_vel)
        
        # 处理急转弯情况 - 关键改进
        if turning_radius < self.min_turning_radius:
            # 对于急转弯，调整到最小转弯半径
            turning_radius = self.min_turning_radius
            
            # 重新计算匹配的线速度或角速度
            if abs(linear_vel) > 0.08:  # 有明显线速度时，调整角速度
                angular_vel = linear_vel / turning_radius * math.copysign(1.0, angular_vel)
            else:  # 低速时，调整线速度以支持掉头
                linear_vel = self.min_turning_radius * abs(angular_vel) * 0.6
                # 保持原有的方向性
                if abs(linear_vel) < 0.05:  # 如果计算出的速度太小
                    linear_vel = math.copysign(0.15, angular_vel)  # 使用最小速度
                else:
                    linear_vel = math.copysign(linear_vel, linear_vel)
        
        # 计算转向角 - 使用精确的阿克曼几何
        if turning_radius > 0:
            steering_angle = math.atan(self.wheelbase / turning_radius)
            steering_angle = math.copysign(steering_angle, angular_vel)
        else:
            steering_angle = 0.0
        
        # 应用转向增益
        steering_angle *= self.steering_gain
        
        # 限制转向角
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        
        # 速度处理 - 支持前进和后退
        final_velocity = max(-self.max_velocity, min(self.max_velocity, linear_vel))
        
        # 特殊处理：掉头时的速度增强
        if abs(steering_angle) > self.max_steering_angle * 0.7:  # 大转向角
            if abs(final_velocity) < 0.1:  # 低速情况
                # 确保有足够的速度来执行掉头
                min_turn_speed = 0.18
                if abs(final_velocity) < min_turn_speed:
                    final_velocity = math.copysign(min_turn_speed, 
                                                 final_velocity if abs(final_velocity) > 0.01 else angular_vel)
        
        return steering_angle, final_velocity

    def basic_ackermann_conversion(self, linear_vel, angular_vel):
        """基础阿克曼转换算法"""
        if abs(linear_vel) < self.min_velocity_threshold:
            return 0.0, 0.0
        
        if abs(angular_vel) < 0.001:
            return 0.0, linear_vel
        
        # 标准转换公式
        tan_steering = (angular_vel * self.wheelbase) / linear_vel
        steering_angle = math.atan(tan_steering) * self.steering_gain
        
        # 限制
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        final_velocity = max(-self.max_velocity, min(self.max_velocity, linear_vel))
        
        return steering_angle, final_velocity

    def apply_steering_filter(self, steering_angle):
        """转向角平滑滤波"""
        filtered_steering = (self.steering_filter_alpha * steering_angle + 
                           (1.0 - self.steering_filter_alpha) * self.previous_steering)
        self.previous_steering = filtered_steering
        return filtered_steering

    def apply_velocity_filter(self, velocity):
        """速度平滑滤波"""
        filtered_velocity = (self.velocity_filter_alpha * velocity + 
                           (1.0 - self.velocity_filter_alpha) * self.previous_velocity)
        self.previous_velocity = filtered_velocity
        return filtered_velocity

    def publish_control_commands(self, steering_angle, velocity):
        """发布控制命令到您的车辆控制器"""
        steering_msg = Float64()
        steering_msg.data = float(steering_angle)
        self.steering_pub.publish(steering_msg)
        
        velocity_msg = Float64()
        velocity_msg.data = float(velocity)
        self.velocity_pub.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = Nav2AckermannConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
