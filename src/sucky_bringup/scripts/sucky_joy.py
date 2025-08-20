#!/usr/bin/env python3

"""
ROS2 Node for handling PS4 controller inputs to control cyclone, doors, shaker, and airlock.

This node subscribes to joystick messages and provides button-triggered
services to toggle the various systems on the Sucky robot.

Subscribed Topics:
  - /joy (sensor_msgs/Joy): Joystick input messages
  - /dust_chamber/status (std_msgs/Bool): Dust chamber full status

Service Clients:
  - /cyclone/set_state (std_srvs/SetBool): Toggle cyclone on/off  
  - /doors/set_state (std_srvs/SetBool): Toggle doors open/closed
  - /shaker/set_state (std_srvs/SetBool): Toggle shaker motor on/off
  - /airlock/set_state (std_srvs/SetBool): Toggle airlock motor on/off
  - /dust_chamber/get_status (std_srvs/Trigger): Get dust chamber status

PS4 Controller Button Mapping:
  - Square (button 0): Toggle cyclone on/off
  - X (button 1): Toggle doors open/closed
  - Circle (button 2): Toggle shaker motor on/off
  - Triangle (button 3): Toggle airlock motor on/off
  - Touchpad (button 13): Status check (log current states)

Parameters:
  - cyclone_button (int): Button index for cyclone toggle (default: 0 - Square)
  - doors_button (int): Button index for doors toggle (default: 1 - X)
  - shaker_button (int): Button index for shaker toggle (default: 2 - Circle)
  - airlock_button (int): Button index for airlock toggle (default: 3 - Triangle)
  - status_button (int): Button index for status check (default: 13 - Touchpad)
  - debounce_time (double): Time in seconds to prevent button spam (default: 0.5)

Usage:
  ros2 run sucky sucky_joy.py
  
  or with parameters:
  ros2 run sucky sucky_joy.py --ros-args -p cyclone_button:=0 -p airlock_button:=3
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
import time


class SuckyJoy(Node):
    def __init__(self):
        super().__init__('sucky_joy')
        
        # Declare parameters for button mapping
        self.declare_parameter('cyclone_button', 0)    # Square button
        self.declare_parameter('doors_button', 1)      # X button  
        self.declare_parameter('shaker_button', 2)     # Circle button
        self.declare_parameter('airlock_button', 3)    # Triangle button
        self.declare_parameter('status_button', 13)    # Touchpad
        self.declare_parameter('debounce_time', 0.5)   # 500ms debounce
        
        # Get parameters
        self.cyclone_button = self.get_parameter('cyclone_button').get_parameter_value().integer_value
        self.doors_button = self.get_parameter('doors_button').get_parameter_value().integer_value
        self.status_button = self.get_parameter('status_button').get_parameter_value().integer_value
        self.shaker_button = self.get_parameter('shaker_button').get_parameter_value().integer_value
        self.airlock_button = self.get_parameter('airlock_button').get_parameter_value().integer_value
        self.debounce_time = self.get_parameter('debounce_time').get_parameter_value().double_value
        
        # State tracking
        self.cyclone_state = False
        self.doors_state = False
        self.shaker_state = False
        self.airlock_state = False
        self.dust_chamber_full = False
        self.last_button_time = {}
        
        # Initialize button press times
        for button in [self.cyclone_button, self.doors_button, self.status_button, self.shaker_button, self.airlock_button]:
            self.last_button_time[button] = 0.0
        
        # Service clients
        self.cyclone_client = self.create_client(SetBool, 'cyclone/set_state')
        self.doors_client = self.create_client(SetBool, 'doors/set_state')
        self.shaker_client = self.create_client(SetBool, 'shaker/set_state')
        self.airlock_client = self.create_client(SetBool, 'airlock/set_state')
        self.dust_chamber_client = self.create_client(Trigger, 'dust_chamber/get_status')
        
        # Dust chamber status subscriber
        self.dust_chamber_sub = self.create_subscription(
            Bool, 'dust_chamber/status', self.dust_chamber_callback, 10)
        
        # Joy subscriber
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        
        # Wait for services
        self.get_logger().info("Waiting for cyclone, doors, shaker, airlock, and dust chamber services...")
        if not self.cyclone_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Cyclone service not available")
        if not self.doors_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Doors service not available")
        if not self.shaker_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Shaker service not available")
        if not self.airlock_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Airlock service not available")
        if not self.dust_chamber_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Dust chamber service not available")
        
        self.get_logger().info("Sucky Joy controller ready!")
        self.get_logger().info(f"Button mapping:")
        self.get_logger().info(f"  Square (button {self.cyclone_button}): Toggle cyclone")
        self.get_logger().info(f"  X (button {self.doors_button}): Toggle doors")
        self.get_logger().info(f"  Circle (button {self.shaker_button}): Toggle shaker")
        self.get_logger().info(f"  Triangle (button {self.airlock_button}): Toggle airlock")
        self.get_logger().info(f"  Touchpad (button {self.status_button}): Status check")

    def joy_callback(self, msg):
        """Handle joystick button presses"""
        current_time = time.time()
        
        # Check if we have enough buttons
        if len(msg.buttons) <= max(self.cyclone_button, self.doors_button, self.status_button, self.shaker_button, self.airlock_button):
            return
        
        # Cyclone toggle (Square button)
        if (msg.buttons[self.cyclone_button] and 
            current_time - self.last_button_time[self.cyclone_button] > self.debounce_time):
            self.last_button_time[self.cyclone_button] = current_time
            self.toggle_cyclone()
        
        # Doors toggle (X button)
        if (msg.buttons[self.doors_button] and 
            current_time - self.last_button_time[self.doors_button] > self.debounce_time):
            self.last_button_time[self.doors_button] = current_time
            self.toggle_doors()
        
        # Shaker toggle (Circle button)
        if (msg.buttons[self.shaker_button] and 
            current_time - self.last_button_time[self.shaker_button] > self.debounce_time):
            self.last_button_time[self.shaker_button] = current_time
            self.toggle_shaker()
        
        # Airlock toggle (Triangle button)
        if (msg.buttons[self.airlock_button] and 
            current_time - self.last_button_time[self.airlock_button] > self.debounce_time):
            self.last_button_time[self.airlock_button] = current_time
            self.toggle_airlock()
        
        # Status check (Touchpad)
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

    def toggle_airlock(self):
        """Toggle airlock state"""
        new_state = not self.airlock_state
        self.get_logger().info(f"Toggling airlock {'ON' if new_state else 'OFF'}")
        
        if self.airlock_client.service_is_ready():
            request = SetBool.Request()
            request.data = new_state
            
            future = self.airlock_client.call_async(request)
            future.add_done_callback(
                lambda f: self.airlock_response_callback(f, new_state))
        else:
            self.get_logger().error("Airlock service not ready")

    def log_status(self):
        """Log current status"""
        self.get_logger().info(f"Current status:")
        self.get_logger().info(f"  Cyclone: {'ON' if self.cyclone_state else 'OFF'}")
        self.get_logger().info(f"  Doors: {'OPEN' if self.doors_state else 'CLOSED'}")
        self.get_logger().info(f"  Shaker: {'ON' if self.shaker_state else 'OFF'}")
        self.get_logger().info(f"  Airlock: {'ON' if self.airlock_state else 'OFF'}")
        self.get_logger().info(f"  Dust Chamber: {'FULL' if self.dust_chamber_full else 'CLEAR'}")

    def dust_chamber_callback(self, msg):
        """Handle dust chamber status updates"""
        old_state = self.dust_chamber_full
        self.dust_chamber_full = msg.data
        
        # Log state changes
        if old_state != self.dust_chamber_full:
            self.get_logger().info(f"Dust chamber status changed: {'FULL' if self.dust_chamber_full else 'CLEAR'}")
            
            # Optional: Add alert for full dust chamber
            if self.dust_chamber_full:
                self.get_logger().warn("⚠️  DUST CHAMBER IS FULL - CONSIDER EMPTYING!")

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

    def airlock_response_callback(self, future, expected_state):
        """Handle airlock service response"""
        try:
            response = future.result()
            if response.success:
                self.airlock_state = expected_state
                self.get_logger().info(f"Airlock successfully {'turned ON' if expected_state else 'turned OFF'}")
            else:
                self.get_logger().error(f"Failed to toggle airlock: {response.message}")
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
