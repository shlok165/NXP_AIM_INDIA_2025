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

import numpy as np
import cv2
import tkinter as tk
from threading import Thread

from sensor_msgs.msg import Joy, CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped
from synapse_msgs.msg import Status, WarehouseShelf

QOS_PROFILE_DEFAULT = 10
PROGRESS_TABLE_GUI = True

class WindowProgressTable:
    def __init__(self, root, shelf_count):
        self.root = root
        self.root.title("Shelf Objects & QR Link")
        self.root.attributes("-topmost", True)

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
        
        self.declare_parameter('shelf_count', 1)
        self.shelf_count = self.get_parameter('shelf_count').get_parameter_value().integer_value

        if PROGRESS_TABLE_GUI:
            self.root = tk.Tk()
            self.progress_table = WindowProgressTable(self.root, self.shelf_count)
            self.gui_thread = Thread(target=self.root.mainloop, daemon=True)
            self.gui_thread.start()

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

    def pose_callback(self, message):
        self.pose_curr = message
        self.buggy_pose_x = message.pose.pose.position.x
        self.buggy_pose_y = message.pose.pose.position.y

    def camera_image_callback(self, message):
        if PROGRESS_TABLE_GUI:
            self.root.update_idletasks()
            self.root.update()
        
        try:
            np_arr = np.frombuffer(message.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # QR code detection
            qr_detector = cv2.QRCodeDetector()
            decoded_text, points, _ = qr_detector.detectAndDecode(image)
            
            if decoded_text and decoded_text != self.qr_code_str:
                self.get_logger().info(f'Detected QR Code: {decoded_text}')
                self.qr_code_str = decoded_text
                
                shelf_data = WarehouseShelf()
                shelf_data.qr_decoded = decoded_text
                
                if PROGRESS_TABLE_GUI:
                    self.progress_table.update_shelf_data(shelf_data)
                    self.root.update_idletasks()
                
                self.publisher_shelf_data.publish(shelf_data)
            
            # Shelf detection
            if self.shelf_detection_enabled:
                shelf_detected = self.detect_shelf(image)
                if shelf_detected:
                    current_time = self.get_clock().now()
                    if True:  # 1 second
                        self.get_logger().info("SHELF DETECTED!")
                        self.last_shelf_detection_time = current_time
            
            # Publish debug images
            if points is not None and len(points) > 0:
                debug_image = image.copy()
                cv2.polylines(debug_image, [np.int32(points)], True, (0, 255, 0), 3)
                
                if decoded_text:
                    cv2.putText(debug_image, decoded_text, 
                               (int(points[0][0][0]), int(points[0][0][1])-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                self.publish_debug_image(self.publisher_qr_decode, debug_image)
            else:
                self.publish_debug_image(self.publisher_qr_decode, image)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

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
            
            self.publish_debug_image(self.publisher_shelf_debug, debug_img)
            return shelf_detected
            
        except Exception as e:
            self.get_logger().error(f'Error in shelf detection: {str(e)}')
            return False

    def publish_debug_image(self, publisher, image):
        if image.size:
            message = CompressedImage()
            _, encoded_data = cv2.imencode('.jpg', image)
            message.format = "jpeg"
            message.data = encoded_data.tobytes()
            publisher.publish(message)

    def cerebri_status_callback(self, message):
        if message.mode == 3 and message.arming == 2:
            self.armed = True
        else:
            msg = Joy()
            msg.buttons = [0, 1, 0, 0, 0, 0, 0, 1]
            msg.axes = [0.0, 0.0, 0.0, 0.0]
            self.publisher_joy.publish(msg)

    def shelf_objects_callback(self, msg):
        try:
            self.shelf_objects_curr = msg
            # self.get_logger().info(f"Received shelf objects: {msg.object_name}")

            if PROGRESS_TABLE_GUI:
                self.progress_table.update_shelf_data(msg)
                self.root.update_idletasks()
        except Exception as e:
            self.get_logger().error(f"Error processing shelf objects: {e}")

    def rover_move_manual_mode(self, speed, turn):
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed, 0.0, turn]
        self.publisher_joy.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    warehouse_explore = WarehouseExplore()
    
    try:
        rclpy.spin(warehouse_explore)
    except Exception as e:
        warehouse_explore.get_logger().error(f"Exception occurred: {e}")
    finally:
        warehouse_explore.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()