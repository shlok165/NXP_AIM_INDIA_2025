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
import signal
import sys

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
        self.root.geometry("900x600")
        self.root.resizable(False, False)
        
        # Handle window close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.headers = ["Shelf", "Objects Detected", "QR Code"]
        self.shelf_count = shelf_count
        self.current_shelf_index = 0
        self.shelf_objects = {}
        self.shelf_qr_codes = {}
        self.all_qr_codes = set()
        self.boxes = []
        self.create_table()
        
    def on_closing(self):
        """Handle window close event"""
        self.root.quit()
        self.root.destroy()

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
        self.current_goal_handle = None
        self.shelf_found = False
        self.node_shutdown = False
        self.declare_parameter('shelf_count', 1)
        self.shelf_count = self.get_parameter('shelf_count').get_parameter_value().integer_value
        self.qr_detected = False
        # Declare alignment parameters
        self.declare_parameter('initial_angle', 0.0)
        self.target_angle_deg = self.get_parameter('initial_angle').get_parameter_value().double_value - 1
        self.target_angle_rad = math.radians(self.target_angle_deg)
        
        # Navigation parameters
        self.step_distance = 2.75  # Distance to move in each step
        self.navigation_active = False
        self.initial_position_set = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        
        # Nav2 Action Client
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        if PROGRESS_TABLE_GUI:
            self.gui_initialized = False
            self.gui_queue = []
            self.gui_thread = None
            self.init_gui()
        
        # Add a timer to test navigation after initialization
        self.create_timer(10.0, self.initial_navigation)

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
            
        self.gui_thread = Thread(target=gui_thread_func, daemon=False)  # Not daemon so it stays alive
        self.gui_thread.start()
    
    def safe_gui_update(self, func, *args):
        """Thread-safe GUI update method"""
        if not PROGRESS_TABLE_GUI:
            return
            
        if hasattr(self, 'gui_initialized') and self.gui_initialized:
            self.root.after(0, func, *args)
        else:
            self.gui_queue.append((func, args))

    def emergency_stop_robot(self):
        try:
            if self.current_goal_handle is not None:
                future = self.current_goal_handle.cancel_goal_async()
                future.add_done_callback(self.cancel_done_callback)
                self.get_logger().info("Attemptng to CANCEL current navigation goal")
            # Send immediate stop command (zero velocity)
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            
            # Publish multiple times to ensure it's received
            for _ in range(10):
                self.publisher_cmd_vel.publish(stop_cmd)
                
            # Also send disarm command if available
            if hasattr(self, 'publisher_joy'):
                disarm_msg = Joy()
                disarm_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]  # All buttons released
                disarm_msg.axes = [0.0, 0.0, 0.0, 0.0]  # All axes neutral
                self.publisher_joy.publish(disarm_msg)
                
            self.get_logger().info("EMERGENCY STOP EXECUTED - Robot stopped immediately")
            
        except Exception as e:
            self.get_logger().error(f"Error in emergency_stop_robot: {e}")
        sleep(0.1)
    def cancel_done_callback(self, future):
        try:
            response = future.result()
            if len(response.goals_canceling) > 0:
                self.get_logger().info("Navigation goal successfully cancelled")
            else:
                self.get_logger().info("No navigation goal to cancel")
        except Exception as e:
            self.get_logger().error(f"Error in cancel_done_callback: {e}")

    def camera_image_callback(self, message):
        if self.node_shutdown:
            return
            
        try:
            np_arr = np.frombuffer(message.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Emergency stop flag
            
            
            # QR code detection
            qr_detector = cv2.QRCodeDetector()
            decoded_text, points, _ = qr_detector.detectAndDecode(image)
            
            if decoded_text and decoded_text != self.qr_code_str:
                
                self.qr_code_str = decoded_text
                self.qr_detected = True
                
                # Store QR data
                self.pending_shelf_data = WarehouseShelf()
                self.pending_shelf_data.qr_decoded = decoded_text
                self.progress_table.shelf_qr_codes[self.progress_table.current_shelf_index] = decoded_text
                
                self.get_logger().info(f"QR DETECTED! EMERGENCY STOP TRIGGERED. QR: {decoded_text}")
                self.emergency_stop_robot()
                return
            # Shelf detection - only if not already found
            if not self.shelf_found:
                shelf_detected = self.detect_shelf(image)
                if shelf_detected:
                    self.shelf_found = True
                    self.get_logger().info("SHELF DETECTED! EMERGENCY STOP TRIGGERED.")
                    self.emergency_stop_robot() 
                    return
                    
                    
            # Set shelf_found flag after any detection
            
        except Exception as e:
            self.get_logger().error(f"Error in camera_image_callback: {e}")

    def initial_navigation(self):
        try:
            # Check if robot is armed before navigation
            if not self.armed:
                self.get_logger().info("Robot not armed. Waiting for arming before navigation.")
                return
                
            # Send first navigation goal
            self.send_next_navigation_goal()
            
        except Exception as e:
            self.get_logger().error(f"Error in initial_navigation: {e}")
            self.emergency_stop_robot()

    def send_next_navigation_goal(self):
        """Send next navigation goal in the target direction"""
        if self.shelf_found or self.navigation_active or self.node_shutdown:
            return
            
        # Calculate next target position
        target_x = self.buggy_pose_x + self.step_distance * math.cos(self.target_angle_rad)
        target_y = self.buggy_pose_y + self.step_distance * math.sin(self.target_angle_rad)
        
        self.get_logger().info(f"Sending navigation goal to ({target_x:.2f}, {target_y:.2f})")
        self.navigate_to_pose(target_x, target_y, self.target_angle_rad)

    def euler_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion with normalization"""
        yaw = self.normalize_angle(yaw)
        
        from geometry_msgs.msg import Quaternion
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        
        # Ensure quaternion is normalized
        length = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        if abs(length - 1.0) > 0.01 and length > 0:
            quat.x /= length
            quat.y /= length
            quat.z /= length
            quat.w /= length
        
        return quat

    def navigate_to_pose(self, x, y, yaw):
        """Navigate to a specific pose"""
        if self.shelf_found or self.navigation_active or self.node_shutdown:
            return False
        
        self.navigation_active = True
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        quat = self.euler_to_quaternion(yaw)
        pose.pose.orientation = quat
        
        # Send the goal
        self.nav_action_client.send_goal_async(
            NavigateToPose.Goal(pose=pose),
            feedback_callback=self.navigation_feedback_callback
        ).add_done_callback(self.nav_response_callback)
        
        return True

    def nav_response_callback(self, future):
        try:
            self.current_goal_handle = future.result()
            
            if not self.current_goal_handle.accepted:
                self.get_logger().info('Navigation goal rejected')
                self.emergency_stop_robot()
                return
                
            self.get_logger().info('Navigation goal accepted')
            
            # Get the result
            result_future = self.current_goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)
            
        except Exception as e:
            self.get_logger().error(f"Error in nav_response_callback: {e}")
            self.emergency_stop_robot()

    def nav_result_callback(self, future):
        try:
            result = future.result().result
            status = future.result().status
            
            self.navigation_active = False
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation goal succeeded')
                # Continue with next goal if no shelf found
                if not self.shelf_found and not self.node_shutdown:
                    self.send_next_navigation_goal()
                else:
                    self.get_logger().info('Navigation completed or shelf found')
                    self.emergency_stop_robot()
            else:
                self.get_logger().info(f'Navigation goal failed with status: {status}')
                self.emergency_stop_robot()
            
        except Exception as e:
            self.get_logger().error(f"Error in nav_result_callback: {e}")
            self.emergency_stop_robot()

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback with rate limiting"""
        if self.shelf_found or self.node_shutdown:
            return
            
        try:
            feedback = feedback_msg.feedback
            current_time = self.get_clock().now()
            
            # Rate limit feedback logging
            if not hasattr(self, 'last_feedback_time') or \
               (current_time - self.last_feedback_time).nanoseconds > 500_000_000:
                self.last_feedback_time = current_time
                
                distance_remaining = feedback.distance_remaining
                # Log progress occasionally
                if distance_remaining > 0:
                    self.get_logger().info(f"Distance remaining: {distance_remaining:.2f}m")

        except Exception as e:
            pass

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (yaw)"""
        if abs(x*x + y*y + z*z + w*w - 1.0) > 0.01:
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
        if self.shelf_found or self.node_shutdown:
            return
            
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
            sleep(5.0)
            self.initial_x = self.buggy_pose_x
            self.initial_y = self.buggy_pose_y
            self.initial_yaw = self.current_yaw
            self.initial_position_set = True
            
            target_x = self.buggy_pose_x + 1.8 * math.cos(self.target_angle_rad + 0.01745)
            target_y = self.buggy_pose_y + 1.8 * math.sin(self.target_angle_rad + 0.01745)
            self.navigate_to_pose(target_x, target_y, self.target_angle_rad)
            
            self.get_logger().info(f"Initial position set: x={self.initial_x:.3f}, y={self.initial_y:.3f}")

    def cerebri_status_callback(self, message):
        if self.node_shutdown:
            return
            
        prev_armed = self.armed
        if message.mode == 3 and message.arming == 2:
            self.armed = True
        else:
            self.armed = False
            
            # Send arming command
            msg = Joy()
            msg.buttons = [0, 1, 0, 0, 0, 0, 0, 1]
            msg.axes = [0.0, 0.0, 0.0, 0.0]
            self.publisher_joy.publish(msg)

    def shelf_objects_callback(self, msg):
        if self.node_shutdown:
            return
            
        try:
            self.shelf_objects_curr = msg
            
            # Use thread-safe update
            if self.shelf_found:
                # Always update the GUI table with new object data first
                self.safe_gui_update(self.progress_table.update_shelf_objects, msg.object_name, msg.object_count)
                
                # Check if we have pending QR data and enough objects
                if hasattr(self, 'pending_shelf_data') and self.pending_shelf_data:
                    # Count total objects in the GUI table dictionary for current shelf
                    total_objects = 0
                    if self.progress_table.current_shelf_index in self.progress_table.shelf_objects:
                        total_objects = sum(self.progress_table.shelf_objects[self.progress_table.current_shelf_index].values())
                    
                    self.get_logger().info(f"Current shelf {self.progress_table.current_shelf_index + 1} has {total_objects} total objects")
                    
                    if total_objects >= 6:
                        # Merge object data with stored QR data
                        self.pending_shelf_data.object_name = msg.object_name
                        self.pending_shelf_data.object_count = msg.object_count
                        
                        # NOW update GUI with QR code (only when publishing)
                        self.safe_gui_update(self.progress_table.update_qr_display, self.progress_table.current_shelf_index)
                        
                        # Publish the complete shelf data
                        self.publisher_shelf_data.publish(self.pending_shelf_data)
                        self.get_logger().info(f"Published shelf data with QR: {self.pending_shelf_data.qr_decoded} and {total_objects} objects")
                        
                        # Unlock the next shelf in GUI
                        if self.progress_table.current_shelf_index < self.shelf_count - 1:
                            next_shelf = self.progress_table.current_shelf_index + 1
                            self.safe_gui_update(self.progress_table.unlock_shelf, next_shelf)
                            self.progress_table.current_shelf_index = next_shelf
                        
                        # Clear pending data after publishing
                        self.pending_shelf_data = None
                        
                        # Move to next shelf exploration
                        self.shelf_found = False
                        self.qr_detected = False
                        self.get_logger().info("Shelf exploration complete. Moving to next shelf.")
                    else:
                        self.get_logger().info(f"QR stored but hidden in GUI. Objects: {total_objects}/6")
                else:
                    # No pending QR data, objects are already updated in GUI above
                    self.get_logger().info("Objects detected but no QR code stored yet")
        
        except Exception as e:
            self.get_logger().error(f"Error in shelf_objects_callback: {e}")

    def detect_shelf(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
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
                    return True
        
        return False

    def destroy_node(self):
        """Clean shutdown of the node"""
        self.node_shutdown = True
        self.get_logger().info("Node shutting down...")
        
        # Stop robot one more time
        self.emergency_stop_robot()
        
        # Call parent destroy
        super().destroy_node()


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutdown signal received. Stopping robot and keeping GUI open...")
    sys.exit(0)


def main(args=None):
    sleep(7.0)
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    rclpy.init(args=args)
    warehouse_explore = WarehouseExplore()
    
    try:
        rclpy.spin(warehouse_explore)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        try:
            warehouse_explore.destroy_node()
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print("ROS node shutdown complete. GUI will remain active.")
        
        # Keep the process alive if GUI is still running
        if PROGRESS_TABLE_GUI and hasattr(warehouse_explore, 'gui_thread'):
            try:
                warehouse_explore.gui_thread.join()
            except:
                pass

if __name__ == '__main__':
    main()