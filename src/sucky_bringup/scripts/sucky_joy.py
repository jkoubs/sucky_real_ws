#!/usr/bin/env python3

"""
ROS2 Node for handling PS4 controller inputs to control cyclone and doors.

This node subscribes to joystick messages and provides button-triggered
services to toggle the cyclone vacuum and doors on the Sucky robot.

Subscribed Topics:
  - /joy (sensor_msgs/Joy): Joystick input messages

Service Clients:
  - /cyclone/set_state (std_srvs/SetBool): Toggle cyclone on/off  
  - /doors/set_state (std_srvs/SetBool): Toggle doors open/closed
  - /shaker/set_state (std_srvs/SetBool): Toggle shaker motor on/off

PS4 Controller Button Mapping:
  - Square (button 0): Toggle cyclone on/off
  - Circle (button 2): Toggle doors open/closed  
  - X (button 1): Status check (log current states)
  - Triangle (button 3): Toggle shaker motor on/off

Parameters:
  - cyclone_button (int): Button index for cyclone toggle (default: 0 - Square)
  - doors_button (int): Button index for doors toggle (default: 2 - Circle)
  - status_button (int): Button index for status check (default: 1 - X)
  - shaker_button (int): Button index for shaker toggle (default: 3 - Triangle)
  - debounce_time (double): Time in seconds to prevent button spam (default: 0.5)

Usage:
  ros2 run sucky_bringup sucky_joy.py
  
  or with parameters:
  ros2 run sucky_bringup sucky_joy.py --ros-args -p cyclone_button:=0 -p shaker_button:=3
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
import time


class SuckyJoy(Node):
    def __init__(self):
        super().__init__('sucky_joy')
        
        # Declare parameters for button mapping
        self.declare_parameter('cyclone_button', 0)    # Square button
        self.declare_parameter('doors_button', 2)      # Circle button  
        self.declare_parameter('status_button', 1)     # X button
        self.declare_parameter('shaker_button', 3)     # Triangle button
        self.declare_parameter('debounce_time', 0.5)   # 500ms debounce
        
        # Get parameters
        self.cyclone_button = self.get_parameter('cyclone_button').get_parameter_value().integer_value
        self.doors_button = self.get_parameter('doors_button').get_parameter_value().integer_value
        self.status_button = self.get_parameter('status_button').get_parameter_value().integer_value
        self.shaker_button = self.get_parameter('shaker_button').get_parameter_value().integer_value
        self.debounce_time = self.get_parameter('debounce_time').get_parameter_value().double_value
        
        # State tracking
        self.cyclone_state = False
        self.doors_state = False
        self.shaker_state = False
        self.last_button_time = {}
        
        # Initialize button press times
        for button in [self.cyclone_button, self.doors_button, self.status_button, self.shaker_button]:
            self.last_button_time[button] = 0.0
        
        # Service clients
        self.cyclone_client = self.create_client(SetBool, 'cyclone/set_state')
        self.doors_client = self.create_client(SetBool, 'doors/set_state')
        self.shaker_client = self.create_client(SetBool, 'shaker/set_state')
        
        # Joy subscriber
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        
        # Wait for services
        self.get_logger().info("Waiting for cyclone, doors, and shaker services...")
        if not self.cyclone_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Cyclone service not available")
        if not self.doors_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Doors service not available")
        if not self.shaker_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Shaker service not available")
        
        self.get_logger().info("Sucky Joy controller ready!")
        self.get_logger().info(f"Button mapping:")
        self.get_logger().info(f"  Square (button {self.cyclone_button}): Toggle cyclone")
        self.get_logger().info(f"  Circle (button {self.doors_button}): Toggle doors")
        self.get_logger().info(f"  X (button {self.status_button}): Status check")
        self.get_logger().info(f"  Triangle (button {self.shaker_button}): Toggle shaker")

    def joy_callback(self, msg):
        """Handle joystick button presses"""
        current_time = time.time()
        
        # Check if we have enough buttons
        if len(msg.buttons) <= max(self.cyclone_button, self.doors_button, self.status_button, self.shaker_button):
            return
        
        # Cyclone toggle (Square button)
        if (msg.buttons[self.cyclone_button] and 
            current_time - self.last_button_time[self.cyclone_button] > self.debounce_time):
            self.last_button_time[self.cyclone_button] = current_time
            self.toggle_cyclone()
        
        # Doors toggle (Circle button)
        if (msg.buttons[self.doors_button] and 
            current_time - self.last_button_time[self.doors_button] > self.debounce_time):
            self.last_button_time[self.doors_button] = current_time
            self.toggle_doors()
        
        # Shaker toggle (Triangle button)
        if (msg.buttons[self.shaker_button] and 
            current_time - self.last_button_time[self.shaker_button] > self.debounce_time):
            self.last_button_time[self.shaker_button] = current_time
            self.toggle_shaker()
        
        # Status check (X button)
        if (msg.buttons[self.status_button] and 
            current_time - self.last_button_time[self.status_button] > self.debounce_time):
            self.last_button_time[self.status_button] = current_time
            self.log_status()

    def toggle_cyclone(self):
        """Toggle cyclone state"""
        new_state = not self.cyclone_state
        self.get_logger().info(f"Toggling cyclone {'ON' if new_state else 'OFF'}")
        
        if self.cyclone_client.service_is_ready():
            request = SetBool.Request()
            request.data = new_state
            
            future = self.cyclone_client.call_async(request)
            future.add_done_callback(
                lambda f: self.cyclone_response_callback(f, new_state))
        else:
            self.get_logger().error("Cyclone service not ready")

    def toggle_doors(self):
        """Toggle doors state"""
        new_state = not self.doors_state
        self.get_logger().info(f"Toggling doors {'OPEN' if new_state else 'CLOSED'}")
        
        if self.doors_client.service_is_ready():
            request = SetBool.Request()
            request.data = new_state
            
            future = self.doors_client.call_async(request)
            future.add_done_callback(
                lambda f: self.doors_response_callback(f, new_state))
        else:
            self.get_logger().error("Doors service not ready")

    def toggle_shaker(self):
        """Toggle shaker state"""
        new_state = not self.shaker_state
        self.get_logger().info(f"Toggling shaker {'ON' if new_state else 'OFF'}")
        
        if self.shaker_client.service_is_ready():
            request = SetBool.Request()
            request.data = new_state
            
            future = self.shaker_client.call_async(request)
            future.add_done_callback(
                lambda f: self.shaker_response_callback(f, new_state))
        else:
            self.get_logger().error("Shaker service not ready")

    def log_status(self):
        """Log current status"""
        self.get_logger().info(f"Current status:")
        self.get_logger().info(f"  Cyclone: {'ON' if self.cyclone_state else 'OFF'}")
        self.get_logger().info(f"  Doors: {'OPEN' if self.doors_state else 'CLOSED'}")
        self.get_logger().info(f"  Shaker: {'ON' if self.shaker_state else 'OFF'}")

    def cyclone_response_callback(self, future, expected_state):
        """Handle cyclone service response"""
        try:
            response = future.result()
            if response.success:
                self.cyclone_state = expected_state
                self.get_logger().info(f"Cyclone successfully {'turned ON' if expected_state else 'turned OFF'}")
            else:
                self.get_logger().error(f"Failed to toggle cyclone: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def doors_response_callback(self, future, expected_state):
        """Handle doors service response"""
        try:
            response = future.result()
            if response.success:
                self.doors_state = expected_state
                self.get_logger().info(f"Doors successfully {'OPENED' if expected_state else 'CLOSED'}")
            else:
                self.get_logger().error(f"Failed to toggle doors: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def shaker_response_callback(self, future, expected_state):
        """Handle shaker service response"""
        try:
            response = future.result()
            if response.success:
                self.shaker_state = expected_state
                self.get_logger().info(f"Shaker successfully {'turned ON' if expected_state else 'turned OFF'}")
            else:
                self.get_logger().error(f"Failed to toggle shaker: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    sucky_joy = None
    try:
        sucky_joy = SuckyJoy()
        rclpy.spin(sucky_joy)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if sucky_joy is not None:
            sucky_joy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
