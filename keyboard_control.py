#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import tty
import termios
import threading
import time

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Publishers
        self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)
        self.velocity_pub = self.create_publisher(Float64, '/velocity', 10)
        
        # Control parameters
        self.max_steering_angle = 0.6108652  # radians (from parameters.yaml)
        self.max_velocity = 2.0  # m/s (from parameters.yaml)
        
        # Current values
        self.current_steering = 0.0
        self.current_velocity = 0.0
        
        # Control increments
        self.steering_increment = 0.1
        self.velocity_increment = 0.2
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_commands)
        
        self.get_logger().info('Keyboard controller started!')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W/S: Forward/Backward')
        self.get_logger().info('  A/D: Left/Right steering')
        self.get_logger().info('  Space: Stop')
        self.get_logger().info('  Q: Quit')
    
    def publish_commands(self):
        # Publish steering angle
        steering_msg = Float64()
        steering_msg.data = self.current_steering
        self.steering_pub.publish(steering_msg)
        
        # Publish velocity
        velocity_msg = Float64()
        velocity_msg.data = self.current_velocity
        self.velocity_pub.publish(velocity_msg)
    
    def update_steering(self, direction):
        if direction == 'left':
            self.current_steering = min(self.current_steering + self.steering_increment, 
                                      self.max_steering_angle)
        elif direction == 'right':
            self.current_steering = max(self.current_steering - self.steering_increment, 
                                      -self.max_steering_angle)
        elif direction == 'center':
            self.current_steering = 0.0
    
    def update_velocity(self, direction):
        if direction == 'forward':
            self.current_velocity = min(self.current_velocity + self.velocity_increment, 
                                      self.max_velocity)
        elif direction == 'backward':
            self.current_velocity = max(self.current_velocity - self.velocity_increment, 
                                      -self.max_velocity)
        elif direction == 'stop':
            self.current_velocity = 0.0

def get_key():
    """Get a single keypress from terminal without pressing Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rclpy.init()
    
    controller = KeyboardController()
    
    # Start ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,))
    spin_thread.daemon = True
    spin_thread.start()
    
    print("\n" + "="*50)
    print("🚗 Ackermann Vehicle Keyboard Controller 🚗")
    print("="*50)
    print("Controls:")
    print("  W/S: Forward/Backward")
    print("  A/D: Left/Right steering")
    print("  Space: Stop")
    print("  Q: Quit")
    print("="*50)
    print("Press keys to control the vehicle...")
    
    try:
        while True:
            key = get_key().lower()
            
            if key == 'q':
                print("\nQuitting...")
                break
            elif key == 'w':
                controller.update_velocity('forward')
                print(f"Forward - Velocity: {controller.current_velocity:.2f} m/s")
            elif key == 's':
                controller.update_velocity('backward')
                print(f"Backward - Velocity: {controller.current_velocity:.2f} m/s")
            elif key == 'a':
                controller.update_steering('left')
                print(f"Left - Steering: {controller.current_steering:.2f} rad")
            elif key == 'd':
                controller.update_steering('right')
                print(f"Right - Steering: {controller.current_steering:.2f} rad")
            elif key == ' ':
                controller.update_velocity('stop')
                controller.update_steering('center')
                print("Stopped - All commands reset to 0")
            elif key == '\x03':  # Ctrl+C
                break
                
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    finally:
        # Clean shutdown
        controller.current_velocity = 0.0
        controller.current_steering = 0.0
        controller.publish_commands()
        time.sleep(0.1)
        
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
