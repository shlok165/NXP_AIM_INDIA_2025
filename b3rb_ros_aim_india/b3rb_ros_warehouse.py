# Copyright 2025 NXP
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from time import sleep
import numpy as np
import cv2
import tkinter as tk
from threading import Thread
import math

from sensor_msgs.msg import Joy, CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from synapse_msgs.msg import Status, WarehouseShelf

QOS_PROFILE_DEFAULT = 10
PROGRESS_TABLE_GUI = True

class WindowProgressTable:
    def __init__(self, root, shelf_count):
        self.root = root
        self.root.title("Shelf Objects & QR Link")
        self.root.attributes("-topmost", True)
        
        # Make the window a fixed size
        self.root.geometry("600x400")
        self.root.resizable(False, False)
        
        self.headers = ["Shelf", "Objects Detected", "QR Code"]
        self.shelf_count = shelf_count
        self.current_shelf_index = 0
        self.shelf_objects = {}
        self.shelf_qr_codes = {}
        self.all_qr_codes = set()
        self.boxes = []
        self.create_table()
        
        print(f"GUI initialized with {shelf_count} shelves. Current active shelf: 1")

    def create_table(self):
        header_row = []
        for col, header in enumerate(self.headers):
            box = tk.Text(self.root, width=15, height=1, wrap=tk.WORD, borderwidth=2,
                         relief="solid", font=("Helvetica", 14, "bold"))
            box.insert(tk.END, header)
            box.grid(row=0, column=col, padx=3, pady=3, sticky="nsew")
            box.config(state="disabled")
            header_row.append(box)
        self.boxes.append(header_row)
        
        for row in range(1, self.shelf_count + 1):
            row_boxes = []
            for col in range(len(self.headers)):
                box = tk.Text(self.root, width=15, height=3, wrap=tk.WORD, borderwidth=1,
                             relief="solid", font=("Helvetica", 12))
                
                if col == 0:
                    box.insert(tk.END, f"Shelf {row}")
                    box.config(state="disabled")
                else:
                    if row == 1:
                        box.insert(tk.END, "Waiting for data...")
                        box.config(bg="lightyellow")
                    else:
                        box.insert(tk.END, "Locked")
                        box.config(bg="lightgray")
                
                box.grid(row=row, column=col, padx=3, pady=3, sticky="nsew")
                row_boxes.append(box)
            self.boxes.append(row_boxes)

        for row in range(self.shelf_count + 1):
            self.root.grid_rowconfigure(row, weight=1)
        for col in range(len(self.headers)):
            self.root.grid_columnconfigure(col, weight=1)

    def update_shelf_data(self, shelf_msg, shelf_index=None):
        try:
            if hasattr(shelf_msg, 'qr_decoded') and shelf_msg.qr_decoded:
                qr_code = shelf_msg.qr_decoded.strip()
                if qr_code:
                    self.process_qr_code(qr_code)
                    return
            
            if hasattr(shelf_msg, 'object_name') and hasattr(shelf_msg, 'object_count'):
                self.update_shelf_objects(shelf_msg.object_name, shelf_msg.object_count)
                
        except Exception as e:
            print(f"Error updating shelf data: {e}")

    def update_shelf_objects(self, object_names, object_counts):
        try:
            if self.current_shelf_index not in self.shelf_objects:
                self.shelf_objects[self.current_shelf_index] = {}
            
            for name, count in zip(object_names, object_counts):
                if name and count > 0:
                    if name in self.shelf_objects[self.current_shelf_index]:
                        self.shelf_objects[self.current_shelf_index][name] = max(
                            self.shelf_objects[self.current_shelf_index][name], count
                        )
                    else:
                        self.shelf_objects[self.current_shelf_index][name] = count
            
            self.update_objects_display(self.current_shelf_index)
            print(f"Updated objects for Shelf {self.current_shelf_index + 1}")
            
        except Exception as e:
            print(f"Error updating shelf objects: {e}")
    
    def process_qr_code(self, qr_code):
        try:
            if qr_code in self.all_qr_codes:
                print(f"QR code '{qr_code}' already exists")
                return False
            
            self.shelf_qr_codes[self.current_shelf_index] = qr_code
            self.all_qr_codes.add(qr_code)
            
            self.update_qr_display(self.current_shelf_index)
            print(f"Added QR code '{qr_code}' to Shelf {self.current_shelf_index + 1}")
            
            if self.current_shelf_index < self.shelf_count - 1:
                self.current_shelf_index += 1
                self.unlock_shelf(self.current_shelf_index)
                print(f"Advanced to Shelf {self.current_shelf_index + 1}")
                return True
            else:
                print("All shelves completed!")
                return False
                
        except Exception as e:
            print(f"Error processing QR code: {e}")
            return False
    
    def unlock_shelf(self, shelf_index):
        try:
            self.boxes[shelf_index + 1][1].config(state="normal")
            self.boxes[shelf_index + 1][1].delete(1.0, tk.END)
            self.boxes[shelf_index + 1][1].insert(tk.END, "Waiting for data...")
            self.boxes[shelf_index + 1][1].config(bg="lightyellow")
            
            self.boxes[shelf_index + 1][2].config(state="normal")
            self.boxes[shelf_index + 1][2].delete(1.0, tk.END)
            self.boxes[shelf_index + 1][2].insert(tk.END, "Scan QR code...")
            self.boxes[shelf_index + 1][2].config(bg="lightyellow")
            
        except Exception as e:
            print(f"Error unlocking shelf: {e}")
    
    def update_objects_display(self, shelf_index):
        try:
            if shelf_index in self.shelf_objects and self.shelf_objects[shelf_index]:
                object_list = []
                for obj_name, count in self.shelf_objects[shelf_index].items():
                    if count > 1:
                        object_list.append(f"{obj_name} x{count}")
                    else:
                        object_list.append(obj_name)
                display_text = "\n".join(object_list)
            else:
                display_text = "No objects detected"
            
            self.boxes[shelf_index + 1][1].config(state="normal")
            self.boxes[shelf_index + 1][1].delete(1.0, tk.END)
            self.boxes[shelf_index + 1][1].insert(tk.END, display_text)
            
            if display_text != "No objects detected" and display_text != "Waiting for data...":
                self.boxes[shelf_index + 1][1].config(bg="lightgreen")
            
            self.boxes[shelf_index + 1][1].config(state="normal")
            
        except Exception as e:
            print(f"Error updating objects display: {e}")
    
    def update_qr_display(self, shelf_index):
        try:
            qr_text = self.shelf_qr_codes.get(shelf_index, "")
            
            self.boxes[shelf_index + 1][2].config(state="normal")
            self.boxes[shelf_index + 1][2].delete(1.0, tk.END)
            self.boxes[shelf_index + 1][2].insert(tk.END, qr_text)
            
            self.boxes[shelf_index + 1][2].config(bg="lightblue")
            self.boxes[shelf_index + 1][2].config(state="normal")
            
        except Exception as e:
            print(f"Error updating QR display: {e}")
    

class WarehouseExplore(Node):
    def __init__(self):
        super().__init__('warehouse_explore')
        self.continuous_forward_active = False
        self.navigation_completed = False
        self.shelf_found = False
        self.forward_speed = 0.75  # m/s
        self.declare_parameter('shelf_count', 1)
        self.shelf_count = self.get_parameter('shelf_count').get_parameter_value().integer_value
        
        # Declare alignment parameters
        self.declare_parameter('initial_angle', 0.0)
        self.target_angle_deg = self.get_parameter('initial_angle').get_parameter_value().double_value
        self.target_angle_rad = math.radians(self.target_angle_deg)
        
        # Nav2 Action Client
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Navigation state variables
        self.navigation_active = False
        self.navigation_goal_handle = None
        self.initial_position_set = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        
        # self.get_logger().info(f"Target alignment angle: {self.target_angle_deg}° ({self.target_angle_rad:.3f} rad)")

        # Replace the GUI initialization with thread-safe version
        if PROGRESS_TABLE_GUI:
            self.gui_initialized = False
            self.gui_queue = []
            self.init_gui()
        
        # Add timer to check Nav2 availability
        self.create_timer(5.0, self.check_nav2_status)

        # Add a timer to test navigation after initialization
        self.create_timer(10.0, self.test_navigation)

        # Alignment control variables
        self.alignment_active = True
        self.alignment_tolerance = 0.00  # radians (~5 degrees) - increased tolerance
        self.angular_velocity_gain = 0.1  # Increased gain
        self.max_angular_velocity = 0.75   # Increased max velocity
        self.min_angular_velocity = 0.5  # Increased minimum velocity
        
        # self.get_logger().info(f"Target alignment angle: {self.target_angle_deg}° ({self.target_angle_rad:.3f} rad)")

        # Shelf detection parameters
        self.shelf_detection_enabled = True
        self.min_shelf_contour_area = 5000
        self.shelf_aspect_ratio_range = (0.3, 3.0)
        self.shelf_solidity_threshold = 0.85
        self.last_shelf_detection_time = self.get_clock().now()

        # Subscribers
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            QOS_PROFILE_DEFAULT)

        self.subscription_status = self.create_subscription(
            Status,
            '/cerebri/out/status',
            self.cerebri_status_callback,
            QOS_PROFILE_DEFAULT)

        self.subscription_shelf_objects = self.create_subscription(
            WarehouseShelf,
            '/shelf_objects',
            self.shelf_objects_callback,
            QOS_PROFILE_DEFAULT)

        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            QOS_PROFILE_DEFAULT)

        # Publishers
        self.publisher_joy = self.create_publisher(
            Joy,
            '/cerebri/in/joy',
            QOS_PROFILE_DEFAULT)

        # Add cmd_vel publisher for direct robot control
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            QOS_PROFILE_DEFAULT)

        self.publisher_qr_decode = self.create_publisher(
            CompressedImage,
            "/debug_images/qr_code",
            QOS_PROFILE_DEFAULT)

        self.publisher_shelf_data = self.create_publisher(
            WarehouseShelf,
            "/shelf_data",
            QOS_PROFILE_DEFAULT)

        self.publisher_shelf_debug = self.create_publisher(
            CompressedImage,
            "/debug_images/shelf_detection",
            QOS_PROFILE_DEFAULT)

        # Robot state
        self.armed = False
        self.logger = self.get_logger()
        self.qr_code_str = "Empty"
        self.pose_curr = None
        self.buggy_pose_x = 0.0
        self.buggy_pose_y = 0.0
        self.current_yaw = 0.0
        self.last_yaw = 0.0
        self.alignment_start_time = None
        self.continuous_forward_active = False

    def init_gui(self):
        """Initialize GUI in a separate thread"""
        def gui_thread_func():
            self.root = tk.Tk()
            self.progress_table = WindowProgressTable(self.root, self.shelf_count)
            self.gui_initialized = True
            
            # Process any queued updates
            while self.gui_queue:
                func, args = self.gui_queue.pop(0)
                func(*args)
                
            self.root.mainloop()
            
        self.gui_thread = Thread(target=gui_thread_func, daemon=True)
        self.gui_thread.start()
    
    def safe_gui_update(self, func, *args):
        """Thread-safe GUI update method"""
        if not PROGRESS_TABLE_GUI:
            return
            
        if hasattr(self, 'gui_initialized') and self.gui_initialized:
            self.root.after(0, func, *args)
        else:
            self.gui_queue.append((func, args))

    def gui_mainloop(self):
        """Run GUI mainloop with error handling"""
        try:
            self.root.mainloop()
        except Exception as e:
            pass
            # self.get_logger().error(f"GUI error: {e}")

    def on_closing(self):
        
        """Handle GUI window closing"""
        if self.root:
            self.root.destroy()
            self.root = None


    def handle_navigation_completed(self):
        self.get_logger().info("Navigation completed - continuing forward movement")
        self.navigation_completed = True
        self.start_forward_movement()
        
    def start_forward_movement(self):
        self.continuous_forward_active = True
        self.forward_speed = 0.5  # m/s
        
        if hasattr(self, 'forward_timer'):
            self.forward_timer.cancel()
        self.forward_timer = self.create_timer(0.1, self.forward_motion_callback)
        self.get_logger().info("forward motion activated")
        
        # Cancel any ongoing navigation and prevent new navigation
        if hasattr(self, '_get_result_future') and self._get_result_future:
            self._get_result_future.cancel()
    
    def forward_motion_callback(self):
        """Timer callback to maintain forward motion"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_shelf_detection_time).nanoseconds / 1e9
        
        if self.continuous_forward_active and not self.shelf_found:
            self.rover_move_cmd_vel(20*self.forward_speed, 0.0)

         
        else:
            
            if hasattr(self, 'forward_timer'):
                
                self.forward_timer.cancel()
                self.rover_move_cmd_vel(0.0, 0.0)  # Make sure to stop the robot
                self.get_logger().info("Continuous forward motion stopped")
                

    def check_nav2_status(self):
        """Check if Nav2 server is available"""
        if self.nav_action_client.wait_for_server(timeout_sec=1.0):
            pass
            # self.get_logger().info("Nav2 server is available")
        else:
            pass
            # self.get_logger().warn("Nav2 server is NOT available!")

    def test_navigation(self):
        """Test navigation by sending a simple goal"""
        if self.continuous_forward_active:
            self.get_logger().info("Skipping navigation - continuous forward movement is active")
            return
            
        if not self.armed:
            # self.get_logger().warn("Robot not armed - skipping test navigation")
            return
            
        if not self.initial_position_set:
            # self.get_logger().warn("Initial position not set - skipping test navigation")
            return
            
        # Navigate to a point 1 meter ahead in the direction of the target angle
        target_x = self.initial_x + 1.0 * math.cos(self.target_angle_rad)
        target_y = self.initial_y + 1.0 * math.sin(self.target_angle_rad)
        
        # self.get_logger().info(f"Testing navigation - moving 1 meter ahead to ({target_x:.2f}, {target_y:.2f})")
        self.navigate_to_pose(target_x, target_y, self.target_angle_rad)

    def euler_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion with normalization"""
        # Normalize the yaw angle first
        yaw = self.normalize_angle(yaw)
        
        from geometry_msgs.msg import Quaternion
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        
        # Ensure quaternion is normalized (should already be, but just to be safe)
        length = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        if abs(length - 1.0) > 0.01 and length > 0:  # Only normalize if needed
            quat.x /= length
            quat.y /= length
            quat.z /= length
            quat.w /= length
        
        return quat

    def navigate_to_pose(self, x, y, yaw):
        """Navigate to a specific pose"""
        # Create the PoseStamped message
        
        if self.continuous_forward_active or self.shelf_found:
            self.get_logger().info("Skipping navigation")
            return False
        
        
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        quat = self.euler_to_quaternion(yaw)
        pose.pose.orientation = quat
        
        # Send the goal
        self.get_logger().info(f"Navigating to ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)")
        self.nav_action_client.send_goal_async(
            NavigateToPose.Goal(pose=pose),
            feedback_callback=self.navigation_feedback_callback
        ).add_done_callback(self.nav_response_callback)
        
        return True

    def nav_response_callback(self, future):
        """Handle the response from the NavigateToPose action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return
        
        if not self.navigation_completed:
            self.get_logger().info('Navigation goal accepted')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle the result of the NavigateToPose action"""
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Navigation succeded')
            self.navigation_completed = True
            self.handle_navigation_completed()
        else:
            self.get_logger().warning(f'Navigation failed with status {status}')
            self.navigation_completed = False
        




    def forward_motion_callback(self):
        """Timer callback to maintain forward motion"""
        if self.continuous_forward_active and not self.shelf_found:
            self.rover_move_cmd_vel(3.0, 0.0)
        else:
            # If shelf found or motion deactivated, stop the timer
            if hasattr(self, 'forward_timer'):
                self.forward_timer.cancel()
                self.get_logger().info("Continuous forward motion stopped")
        

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback with rate limiting"""
        try:
            feedback = feedback_msg.feedback
            current_time = self.get_clock().now()
            
            # Rate limit feedback logging to avoid flooding
            if not hasattr(self, 'last_feedback_time') or \
               (current_time - self.last_feedback_time).nanoseconds > 500_000_000:  # 0.5 seconds
                self.last_feedback_time = current_time
                
                # Extract useful information from feedback
                distance_remaining = feedback.distance_remaining
                
                # Get current pose from feedback
                current_pose = feedback.current_pose.pose
                current_x = current_pose.position.x
                current_y = current_pose.position.y
                
                # Extract orientation as yaw angle
                orientation = current_pose.orientation
                current_yaw = self.quaternion_to_euler(
                    orientation.x, orientation.y, orientation.z, orientation.w
                )

        except Exception as e:
            pass

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (yaw) with improved robustness"""
        # Check for degenerate cases
        if abs(x*x + y*y + z*z + w*w - 1.0) > 0.01:
            # Not normalized, attempt to normalize
            norm = math.sqrt(x*x + y*y + z*z + w*w)
            if norm > 0:
                x /= norm
                y /= norm
                z /= norm
                w /= norm
        
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return self.normalize_angle(yaw)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def pose_callback(self, message):
        self.pose_curr = message
        self.buggy_pose_x = message.pose.pose.position.x
        self.buggy_pose_y = message.pose.pose.position.y
        
        # Extract current yaw from quaternion
        orientation = message.pose.pose.orientation
        self.current_yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        # Store initial position when first pose is received
        if not self.initial_position_set:
            self.initial_x = self.buggy_pose_x
            self.initial_y = self.buggy_pose_y
            self.initial_yaw = self.current_yaw
            self.initial_position_set = True
            # self.get_logger().info(f"Initial position set: x={self.initial_x:.3f}, y={self.initial_y:.3f}, yaw={math.degrees(self.initial_yaw):.1f}°")

    

    def perform_alignment(self):
        """Align the robot to the target angle"""
        if self.alignment_start_time is None:
            self.alignment_start_time = self.get_clock().now()
        
        # Calculate angle difference
        angle_diff = self.normalize_angle(self.target_angle_rad - self.current_yaw)
        
        # Check if alignment is complete
        if abs(angle_diff) < self.alignment_tolerance:
            if self.alignment_active:
                # self.get_logger().info(f"Alignment complete! Current angle: {math.degrees(self.current_yaw):.1f}°")
                self.alignment_active = False
                # Stop the robot
                self.rover_move_cmd_vel(0.0, 0.0)
            return
        
        # Check for timeout (30 seconds)
        elapsed_time = (self.get_clock().now() - self.alignment_start_time).nanoseconds / 1e9
        if elapsed_time > 30.0:
            # self.get_logger().warn("Alignment timeout reached, stopping alignment")
            self.alignment_active = False
            self.rover_move_cmd_vel(0.0, 0.0)
            return
        
        # For steering drive: need both linear and angular velocity
        # Calculate angular velocity with proportional control
        angular_velocity = self.angular_velocity_gain * angle_diff
        
        # Limit angular velocity
        angular_velocity = max(min(angular_velocity, self.max_angular_velocity), -self.max_angular_velocity)
        
        # Apply minimum angular velocity to overcome static friction
        if abs(angular_velocity) < self.min_angular_velocity:
            angular_velocity = self.min_angular_velocity if angular_velocity > 0 else -self.min_angular_velocity
        
        # For steering drive, provide small forward velocity to enable turning
        linear_velocity = 0.5  # Small forward velocity to enable steering
        
        # Send movement command using both methods
        self.rover_move_cmd_vel(linear_velocity, angular_velocity)
        self.rover_move_manual_mode(linear_velocity, angular_velocity)
        
        # # Log progress
        # self.get_logger().info(f"Aligning: Current={math.degrees(self.current_yaw):.1f}°, "
        #                       f"Target={self.target_angle_deg:.1f}°, "
        #                       f"Error={math.degrees(angle_diff):.1f}°, "
        #                       f"LinVel={linear_velocity:.3f}, AngVel={angular_velocity:.3f}")

    def rover_move_cmd_vel(self, linear_velocity, angular_velocity):
        """Send movement command using Twist message on /cmd_vel topic"""
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.publisher_cmd_vel.publish(twist)

    def camera_image_callback(self, message):
        try:
            np_arr = np.frombuffer(message.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # QR code detection
            qr_detector = cv2.QRCodeDetector()
            decoded_text, points, _ = qr_detector.detectAndDecode(image)
            
            if decoded_text and decoded_text != self.qr_code_str:
                # self.get_logger().info(f'Detected QR Code: {decoded_text}')
                self.qr_code_str = decoded_text
                
                shelf_data = WarehouseShelf()
                shelf_data.qr_decoded = decoded_text
                
                # Use thread-safe update
                self.safe_gui_update(self.progress_table.update_shelf_data, shelf_data)
                
                self.publisher_shelf_data.publish(shelf_data)
            
            
            shelf_detected = self.detect_shelf(image)
            if shelf_detected:    
                self.get_logger().info("SHELF DETECTED!")
                self.rover_move_cmd_vel(0.0, 0.0)  # Stop the robot
                self.get_logger().info("Stopping robot for shelf detection")
                self.shelf_found = True
                self.continuous_forward_active = False
  

            
        except Exception as e:
            pass
            # self.get_logger().error(f'Error processing image: {str(e)}')

    def cerebri_status_callback(self, message):
        prev_armed = self.armed
        if message.mode == 3 and message.arming == 2:
            self.armed = True
            if not prev_armed:
                pass
                # self.get_logger().info("Robot armed - starting alignment")
                # You could directly initiate navigation here if needed
        else:
            self.armed = False
            if prev_armed:
                pass
                # self.get_logger().info("Robot disarmed - cancelling navigation")
                # Cancel any active navigation
            
            # Send arming command
            msg = Joy()
            msg.buttons = [0, 1, 0, 0, 0, 0, 0, 1]
            msg.axes = [0.0, 0.0, 0.0, 0.0]
            self.publisher_joy.publish(msg)

    def shelf_objects_callback(self, msg):
        try:
            self.shelf_objects_curr = msg
            # self.get_logger().info(f"Received shelf objects: {msg.object_name}")
            
            # Use thread-safe update
            if self.shelf_found:
                self.safe_gui_update(self.progress_table.update_shelf_data, msg)
            
        except Exception as e:
            pass
            # self.get_logger().error(f"Error processing shelf objects: {e}")

    def rover_move_manual_mode(self, speed, turn):
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed, 0.0, turn]
        self.publisher_joy.publish(msg)
            
        
    def detect_shelf(self, image):
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            
            contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            debug_img = image.copy()
            shelf_detected = False
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_shelf_contour_area:
                    continue
                
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w)/h
                    
                    hull = cv2.convexHull(contour)
                    hull_area = cv2.contourArea(hull)
                    solidity = float(area)/hull_area if hull_area > 0 else 0
                    
                    if (self.shelf_aspect_ratio_range[0] < aspect_ratio < self.shelf_aspect_ratio_range[1] and
                        solidity > self.shelf_solidity_threshold):
                        
                        shelf_detected = True
                        cv2.drawContours(debug_img, [contour], -1, (0, 255, 0), 2)
                        cv2.putText(debug_img, "Shelf", (x, y-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            
            return shelf_detected
            
        except Exception as e:
            self.get_logger().error(f'Error in shelf detection: {str(e)}')
            return False
        
    def timer_callback(self):
        if self.navigation_completed and not self.shelf_found:
            self.start_forward_movement()
    
    

def main(args=None):
    # a sleep timer for 7 seconds to let initialize things properly
    sleep(7.0)

    rclpy.init(args=args)
    warehouse_explore = WarehouseExplore()
    
    try:
        rclpy.spin(warehouse_explore)
    except Exception as e:
        pass
        # warehouse_explore.get_logger().error(f"Exception occurred: {e}")
    finally:
        warehouse_explore.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()