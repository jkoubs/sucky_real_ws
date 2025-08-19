#!/usr/bin/env python3

"""
ROS2 Node for controlling cyclone vacuum, door servos, shaker motor, and airlock motor via Arduino over serial.

This unified controller handles all system operations through a single
serial connection to the Arduino, preventing communication conflicts that occur
when multiple nodes try to access the same serial port.

Published Topics:
  - /cyclone/status (std_msgs/Bool): Current cyclone state (True=ON, False=OFF)
  - /doors/status (std_msgs/Bool): Current door state (True=OPEN, False=CLOSED)
  - /shaker/status (std_msgs/Bool): Current shaker state (True=ON, False=OFF)
  - /airlock/status (std_msgs/Bool): Current airlock state (True=ON, False=OFF)
  - /dust_chamber/status (std_msgs/Bool): Dust chamber full status (True=FULL, False=CLEAR)

Services:
  - /cyclone/set_state (std_srvs/SetBool): Turn cyclone on (True) or off (False)
  - /cyclone/get_status (std_srvs/Trigger): Get current cyclone status
  - /doors/set_state (std_srvs/SetBool): Open (True) or close (False) the doors
  - /doors/get_status (std_srvs/Trigger): Get current door status
  - /shaker/set_state (std_srvs/SetBool): Turn shaker on (True) or off (False)
  - /shaker/get_status (std_srvs/Trigger): Get current shaker status
  - /airlock/set_state (std_srvs/SetBool): Turn airlock on (True) or off (False)
  - /airlock/get_status (std_srvs/Trigger): Get current airlock status
  - /dust_chamber/get_status (std_srvs/Trigger): Get current dust chamber status

Parameters:
  - serial_port (string): Arduino serial port (default: /dev/ttyACM0)
  - baud_rate (int): Serial communication baud rate (default: 115200)
  - timeout (double): Serial communication timeout (default: 2.0)

Arduino Commands:
  - CYCLONE_ON: Turn cyclone on
  - CYCLONE_OFF: Turn cyclone off
  - DOOR_OPEN: Opens both doors (left servo to 0°, right servo to 180°)
  - DOOR_CLOSE: Closes both doors (left servo to 180°, right servo to 0°)
  - SHAKER_ON: Turn shaker motor on
  - SHAKER_OFF: Turn shaker motor off
  - AIRLOCK_ON: Turn airlock motor on
  - AIRLOCK_OFF: Turn airlock motor off
  - STATUS: Returns status of cyclone, doors, shaker, airlock, and dust chamber

Usage:
  ros2 run sucky arduino_controller.py
  
  or with parameters:
  ros2 run sucky arduino_controller.py --ros-args -p serial_port:=/dev/ttyACM0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
import serial
import time
import threading


class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 2.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # Serial connection and state
        self.serial_conn = None
        self.cyclone_state = False
        self.doors_open = False
        self.shaker_state = False
        self.airlock_state = False
        self.dust_chamber_full = False
        self.serial_lock = threading.Lock()  # Prevent concurrent serial access
        
        # Serial monitoring thread
        self.monitoring_active = False
        self.monitor_thread = None
        
        # Publishers
        self.cyclone_status_pub = self.create_publisher(Bool, 'cyclone/status', 10)
        self.doors_status_pub = self.create_publisher(Bool, 'doors/status', 10)
        self.shaker_status_pub = self.create_publisher(Bool, 'shaker/status', 10)
        self.airlock_status_pub = self.create_publisher(Bool, 'airlock/status', 10)
        self.dust_chamber_status_pub = self.create_publisher(Bool, 'dust_chamber/status', 10)
        
        # Cyclone services
        self.set_cyclone_srv = self.create_service(
            SetBool, 'cyclone/set_state', self.set_cyclone_callback)
        self.get_cyclone_status_srv = self.create_service(
            Trigger, 'cyclone/get_status', self.get_cyclone_status_callback)
        
        # Door services
        self.set_doors_srv = self.create_service(
            SetBool, 'doors/set_state', self.set_doors_callback)
        self.get_doors_status_srv = self.create_service(
            Trigger, 'doors/get_status', self.get_doors_status_callback)
        
        # Shaker services
        self.set_shaker_srv = self.create_service(
            SetBool, 'shaker/set_state', self.set_shaker_callback)
        self.get_shaker_status_srv = self.create_service(
            Trigger, 'shaker/get_status', self.get_shaker_status_callback)
        
        # Airlock services
        self.set_airlock_srv = self.create_service(
            SetBool, 'airlock/set_state', self.set_airlock_callback)
        self.get_airlock_status_srv = self.create_service(
            Trigger, 'airlock/get_status', self.get_airlock_status_callback)
        
        # Dust chamber services
        self.get_dust_chamber_status_srv = self.create_service(
            Trigger, 'dust_chamber/get_status', self.get_dust_chamber_status_callback)
        
        # Status publishing timer (every 2 seconds)
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Initialize serial connection
        self.connect_serial()
        
        self.get_logger().info(f"Arduino controller ready on {self.serial_port}")
        self.get_logger().info("Services available: cyclone, doors, shaker, airlock, and dust chamber control")
        self.get_logger().info("Controlling all systems through unified interface")

    def connect_serial(self):
        """Establish serial connection to Arduino"""
        try:
            self.get_logger().info(f"Connecting to Arduino on {self.serial_port}...")
            
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )
                
                self.get_logger().info("Serial port opened, waiting for Arduino initialization...")
                
                # Wait for Arduino ready signal
                ready_received = False
                start_time = time.time()
                while not ready_received and (time.time() - start_time) < 10.0:
                    if self.serial_conn.in_waiting > 0:
                        try:
                            line = self.serial_conn.readline().decode().strip()
                            if line == "ARDUINO_READY":
                                ready_received = True
                                self.get_logger().info("Arduino ready signal received")
                            elif line:
                                self.get_logger().debug(f"Arduino startup: {line}")
                        except UnicodeDecodeError:
                            pass
                    time.sleep(0.1)
                
                if not ready_received:
                    self.get_logger().warn("Arduino ready signal not received, proceeding anyway")
                
                # Get initial status
                status_response = self.send_command_unsafe("STATUS", extended_timeout=False)
                if status_response:
                    self.get_logger().info("Initial status received:")
                    for line in status_response.split('\n'):
                        if line.strip():
                            self.get_logger().info(f"  {line.strip()}")
                    self.parse_status_response(status_response)
                else:
                    self.get_logger().info("No initial status received, using defaults")
                
                self.get_logger().info("Connected successfully!")
                self.get_logger().info(f"  Initial cyclone state: {'ON' if self.cyclone_state else 'OFF'}")
                self.get_logger().info(f"  Initial door state: {'OPEN' if self.doors_open else 'CLOSED'}")
                self.get_logger().info(f"  Initial shaker state: {'ON' if self.shaker_state else 'OFF'}")
                self.get_logger().info(f"  Initial airlock state: {'ON' if self.airlock_state else 'OFF'}")
                self.get_logger().info(f"  Initial dust chamber state: {'FULL' if self.dust_chamber_full else 'CLEAR'}")
                
                # Start monitoring thread for real-time Arduino messages
                self.start_monitoring()
                
                return True
                
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            return False

    def send_command_unsafe(self, command, extended_timeout=False):
        """Send command to Arduino without lock (for internal use when lock is already held)"""
        try:
            if not self.serial_conn or not self.serial_conn.is_open:
                self.get_logger().error("Serial connection not available")
                return None
            
            self.get_logger().debug(f"Sending command: {command}")
            
            # Send command
            cmd = f"{command}\n"
            self.serial_conn.write(cmd.encode())
            self.serial_conn.flush()
            
            # Wait for response with extended timeout for cyclone operations
            initial_wait = 0.3
            timeout = 5.0 if extended_timeout else 2.0
            
            time.sleep(initial_wait)
            
            response_lines = []
            start_time = time.time()
            
            # Read response with timeout
            while (time.time() - start_time) < timeout:
                if self.serial_conn.in_waiting > 0:
                    try:
                        line = self.serial_conn.readline().decode().strip()
                        if line:
                            response_lines.append(line)
                            self.get_logger().debug(f"Arduino response: {line}")
                            
                            # For cyclone commands, continue reading until completion
                            if extended_timeout and ("CYCLONE_STARTING" in line or "CYCLONE_STOPPING" in line):
                                # Got acknowledgment, continue reading for completion
                                continue
                            elif extended_timeout and ("Cyclone ON" in line or "Cyclone OFF" in line):
                                # Got completion, we can return
                                break
                                
                    except UnicodeDecodeError as e:
                        self.get_logger().warn(f"Failed to decode Arduino response: {e}")
                        continue
                else:
                    # No more data available, wait a bit and check again
                    time.sleep(0.1)
                    if self.serial_conn.in_waiting == 0 and not extended_timeout:
                        break
                    elif extended_timeout and response_lines:
                        # For cyclone commands, if we got any response, keep waiting for completion
                        continue
                    elif not extended_timeout:
                        break
            
            if response_lines:
                return "\n".join(response_lines)
            else:
                self.get_logger().warn(f"No response from Arduino for command: {command}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")
            return None

    def send_command(self, command, extended_timeout=False, retry_count=2):
        """Send command to Arduino with thread safety and retry logic"""
        with self.serial_lock:
            for attempt in range(retry_count + 1):
                if attempt > 0:
                    self.get_logger().info(f"Retrying command '{command}' (attempt {attempt + 1})")
                    time.sleep(0.5)  # Brief delay before retry
                
                result = self.send_command_unsafe(command, extended_timeout)
                if result:
                    return result
                    
            self.get_logger().error(f"Command '{command}' failed after {retry_count + 1} attempts")
            return None

    def parse_status_response(self, response):
        """Parse Arduino STATUS response and update internal state"""
        if not response:
            self.get_logger().debug("No status response to parse - using default states")
            return
        
        # Parse cyclone state - handle both completion and transitional states
        if "Cyclone: ON" in response:
            self.cyclone_state = True
        elif "Cyclone: OFF" in response:
            self.cyclone_state = False
        elif "CYCLONE_STARTING" in response:
            self.cyclone_state = True  # Transitioning to ON
        elif "CYCLONE_STOPPING" in response:
            self.cyclone_state = False  # Transitioning to OFF
        
        # Parse door state  
        if "Doors: OPEN" in response:
            self.doors_open = True
        elif "Doors: CLOSED" in response:
            self.doors_open = False
        
        # Parse shaker state
        if "Shaker: ON" in response:
            self.shaker_state = True
        elif "Shaker: OFF" in response:
            self.shaker_state = False

        # Parse airlock state
        if "Airlock: ON" in response:
            self.airlock_state = True
        elif "Airlock: OFF" in response:
            self.airlock_state = False

        # Parse dust chamber state
        if "Dust Chamber: FULL" in response:
            self.dust_chamber_full = True
        elif "Dust Chamber: CLEAR" in response:
            self.dust_chamber_full = False

    def set_cyclone_callback(self, request, response):
        """Service callback to set cyclone state"""
        self.get_logger().info(f"Setting cyclone to: {'ON' if request.data else 'OFF'}")
        
        try:
            command = "CYCLONE_ON" if request.data else "CYCLONE_OFF"
            # Use extended timeout and retry for cyclone commands
            result = self.send_command(command, extended_timeout=True, retry_count=2)
            
            if result:
                # Check for success in response - look for both acknowledgment and completion
                if request.data and (
                    "Cyclone ON" in result or 
                    "Cyclone already ON" in result or 
                    "CYCLONE_STARTING" in result
                ):
                    self.cyclone_state = True
                    response.success = True
                    response.message = "Cyclone turned ON"
                elif not request.data and (
                    "Cyclone OFF" in result or 
                    "Cyclone already OFF" in result or 
                    "CYCLONE_STOPPING" in result
                ):
                    self.cyclone_state = False
                    response.success = True
                    response.message = "Cyclone turned OFF"
                else:
                    response.success = False
                    response.message = f"Unexpected response: {result}"
            else:
                response.success = False
                response.message = "No response from Arduino after retries"
                
            self.get_logger().info(f"Cyclone control result: {response.message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def set_doors_callback(self, request, response):
        """Service callback to set door state"""
        self.get_logger().info(f"Setting doors to: {'OPEN' if request.data else 'CLOSED'}")
        
        try:
            command = "DOOR_OPEN" if request.data else "DOOR_CLOSE"
            result = self.send_command(command)
            
            if result:
                # Check for success in response - look for completion messages
                if request.data and ("Doors OPEN" in result or "opening" in result or "Doors already OPEN" in result):
                    self.doors_open = True
                    response.success = True
                    response.message = "Doors OPENED"
                elif not request.data and ("Doors CLOSED" in result or "closing" in result or "Doors already CLOSED" in result):
                    self.doors_open = False
                    response.success = True
                    response.message = "Doors CLOSED"
                else:
                    response.success = False
                    response.message = f"Unexpected response: {result}"
            else:
                response.success = False
                response.message = "No response from Arduino"
                
            self.get_logger().info(f"Door control result: {response.message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def get_cyclone_status_callback(self, request, response):
        """Service callback to get cyclone status"""
        self.get_logger().info("Cyclone status request received")
        
        try:
            result = self.send_command("STATUS")
            
            if result:
                self.parse_status_response(result)
                response.success = True
                response.message = f"Cyclone is {'ON' if self.cyclone_state else 'OFF'}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "Failed to communicate with Arduino"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def get_doors_status_callback(self, request, response):
        """Service callback to get door status"""
        self.get_logger().info("Door status request received")
        
        try:
            result = self.send_command("STATUS")
            
            if result:
                self.parse_status_response(result)
                response.success = True
                response.message = f"Doors are {'OPEN' if self.doors_open else 'CLOSED'}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "Failed to communicate with Arduino"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def set_shaker_callback(self, request, response):
        """Service callback to set shaker state"""
        self.get_logger().info(f"Setting shaker to: {'ON' if request.data else 'OFF'}")
        
        try:
            command = "SHAKER_ON" if request.data else "SHAKER_OFF"
            result = self.send_command(command)
            
            if result:
                # Check for success in response
                if request.data and ("Shaker ON" in result or "Shaker already ON" in result):
                    self.shaker_state = True
                    response.success = True
                    response.message = "Shaker turned ON"
                elif not request.data and ("Shaker OFF" in result or "Shaker already OFF" in result):
                    self.shaker_state = False
                    response.success = True
                    response.message = "Shaker turned OFF"
                else:
                    response.success = False
                    response.message = f"Unexpected response: {result}"
            else:
                response.success = False
                response.message = "No response from Arduino"
                
            self.get_logger().info(f"Shaker control result: {response.message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def get_shaker_status_callback(self, request, response):
        """Service callback to get shaker status"""
        self.get_logger().info("Shaker status request received")
        
        try:
            result = self.send_command("STATUS")
            
            if result:
                self.parse_status_response(result)
                response.success = True
                response.message = f"Shaker is {'ON' if self.shaker_state else 'OFF'}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "Failed to communicate with Arduino"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def set_airlock_callback(self, request, response):
        """Service callback to set airlock state"""
        self.get_logger().info(f"Setting airlock to: {'ON' if request.data else 'OFF'}")
        
        try:
            command = "AIRLOCK_ON" if request.data else "AIRLOCK_OFF"
            result = self.send_command(command)
            
            if result:
                # Check for success in response
                if request.data and ("Airlock ON" in result or "Airlock already ON" in result):
                    self.airlock_state = True
                    response.success = True
                    response.message = "Airlock turned ON"
                elif not request.data and ("Airlock OFF" in result or "Airlock already OFF" in result):
                    self.airlock_state = False
                    response.success = True
                    response.message = "Airlock turned OFF"
                else:
                    response.success = False
                    response.message = f"Unexpected response: {result}"
            else:
                response.success = False
                response.message = "No response from Arduino"
                
            self.get_logger().info(f"Airlock control result: {response.message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def get_airlock_status_callback(self, request, response):
        """Service callback to get airlock status"""
        self.get_logger().info("Airlock status request received")
        
        try:
            result = self.send_command("STATUS")
            
            if result:
                self.parse_status_response(result)
                response.success = True
                response.message = f"Airlock is {'ON' if self.airlock_state else 'OFF'}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "Failed to communicate with Arduino"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def get_dust_chamber_status_callback(self, request, response):
        """Service callback to get dust chamber status"""
        self.get_logger().info("Dust chamber status request received")
        
        try:
            result = self.send_command("STATUS")
            
            if result:
                self.parse_status_response(result)
                response.success = True
                response.message = f"Dust chamber is {'FULL' if self.dust_chamber_full else 'CLEAR'}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "Failed to communicate with Arduino"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def publish_status(self):
        """Publish current status for cyclone, doors, shaker, airlock, and dust chamber"""
        # Publish cyclone status
        cyclone_msg = Bool()
        cyclone_msg.data = self.cyclone_state
        self.cyclone_status_pub.publish(cyclone_msg)
        
        # Publish door status
        doors_msg = Bool()
        doors_msg.data = self.doors_open
        self.doors_status_pub.publish(doors_msg)
        
        # Publish shaker status
        shaker_msg = Bool()
        shaker_msg.data = self.shaker_state
        self.shaker_status_pub.publish(shaker_msg)
        
        # Publish airlock status
        airlock_msg = Bool()
        airlock_msg.data = self.airlock_state
        self.airlock_status_pub.publish(airlock_msg)
        
        # Publish dust chamber status
        dust_chamber_msg = Bool()
        dust_chamber_msg.data = self.dust_chamber_full
        self.dust_chamber_status_pub.publish(dust_chamber_msg)

    def start_monitoring(self):
        """Start background thread to monitor Arduino messages"""
        if not self.monitoring_active:
            self.monitoring_active = True
            self.monitor_thread = threading.Thread(target=self._monitor_arduino, daemon=True)
            self.monitor_thread.start()
            self.get_logger().info("Arduino monitoring thread started")

    def stop_monitoring(self):
        """Stop background monitoring thread"""
        if self.monitoring_active:
            self.monitoring_active = False
            if self.monitor_thread and self.monitor_thread.is_alive():
                self.monitor_thread.join(timeout=1.0)
            self.get_logger().info("Arduino monitoring thread stopped")

    def _monitor_arduino(self):
        """Background thread function to continuously monitor Arduino messages"""
        self.get_logger().info("Arduino monitoring thread running")
        
        while self.monitoring_active:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    # Check for incoming data without blocking service calls
                    if self.serial_conn.in_waiting > 0:
                        # Only acquire lock briefly to read data
                        with self.serial_lock:
                            if self.serial_conn.in_waiting > 0:
                                try:
                                    line = self.serial_conn.readline().decode().strip()
                                    if line:
                                        self._process_arduino_message(line)
                                except UnicodeDecodeError:
                                    pass  # Ignore decode errors
                    else:
                        # No data available, sleep briefly
                        time.sleep(0.05)
                else:
                    # Serial not available, sleep longer
                    time.sleep(0.5)
                    
            except Exception as e:
                if self.monitoring_active:  # Only log if we're supposed to be running
                    self.get_logger().error(f"Arduino monitoring error: {e}")
                time.sleep(0.5)
                
        self.get_logger().info("Arduino monitoring thread exited")

    def _process_arduino_message(self, message):
        """Process a single message from Arduino"""
        self.get_logger().debug(f"Arduino message: {message}")
        
        # Process dust chamber status changes
        if "Dust chamber: FULL" in message:
            if not self.dust_chamber_full:
                self.dust_chamber_full = True
                self.get_logger().info("Dust chamber is now FULL")
        elif "Dust chamber: CLEAR" in message:
            if self.dust_chamber_full:
                self.dust_chamber_full = False
                self.get_logger().info("Dust chamber is now CLEAR")
        
        # Process other real-time status messages
        elif "Cyclone ON" in message:
            self.cyclone_state = True
        elif "Cyclone OFF" in message:
            self.cyclone_state = False
        elif "Doors OPEN" in message:
            self.doors_open = True
        elif "Doors CLOSED" in message:
            self.doors_open = False
        elif "Shaker ON" in message:
            self.shaker_state = True
        elif "Shaker OFF" in message:
            self.shaker_state = False
        elif "Airlock ON" in message:
            self.airlock_state = True
        elif "Airlock OFF" in message:
            self.airlock_state = False

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Arduino controller...")
        
        # Stop monitoring thread first
        self.stop_monitoring()
        
        with self.serial_lock:
            # Turn off cyclone and close doors before shutdown for safety
            if self.cyclone_state:
                self.get_logger().info("Turning off cyclone before shutdown...")
                self.send_command_unsafe("CYCLONE_OFF", extended_timeout=True)
            
            if self.doors_open:
                self.get_logger().info("Closing doors before shutdown...")
                self.send_command_unsafe("DOOR_CLOSE")
            
            if self.shaker_state:
                self.get_logger().info("Turning off shaker before shutdown...")
                self.send_command_unsafe("SHAKER_OFF")
            
            if self.airlock_state:
                self.get_logger().info("Turning off airlock before shutdown...")
                self.send_command_unsafe("AIRLOCK_OFF")
            
            # Close serial connection
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                self.get_logger().info("Serial connection closed")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    arduino_controller = None
    try:
        arduino_controller = ArduinoController()
        rclpy.spin(arduino_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if arduino_controller is not None:
            arduino_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
