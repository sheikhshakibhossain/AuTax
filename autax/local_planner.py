import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import os
import sys

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        self.publisher = self.create_publisher(CompressedImage, '/webcam_image/compressed', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/vision/cmd_vel', 10)

        # Camera setup
        self.camera_model = 'C270'
        self.cap = cv2.VideoCapture()
        self.cap.open(self.find_camera_interface(self.camera_model))

        # Object detection setup
        self.classNames = []
        home_dir = os.path.expanduser("~")
        base_path = os.path.join(home_dir, "ros2_ws/src/AuTax/resource")

        classFile = os.path.join(base_path, "coco.names")
        with open(classFile, "rt") as f:
            self.classNames = f.read().rstrip("\n").split("\n")

        configPath = os.path.join(base_path, "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt")
        weightsPath = os.path.join(base_path, "frozen_inference_graph.pb")
        
        self.net = cv2.dnn_DetectionModel(weightsPath, configPath)
        self.net.setInputSize(320, 320)
        self.net.setInputScale(1.0 / 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)

        # Set detection threshold
        self.detection_threshold = 0.6  # 70% confidence threshold

        # Create a timer to periodically process frames
        self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info('Object Detection Node Started')

    def find_camera_interface(self, key):
        folder_path = '/dev/v4l/by-id/'
        interface = None
        if os.path.exists(folder_path):
            file_names = os.listdir(folder_path)

            for file_name in file_names:
                if key in file_name:
                    interface = folder_path + file_name
                    interface = list(interface)
                    interface[-1] = '0'
                    interface = ''.join(interface)
                    interface = interface.strip()
                    self.get_logger().info(f"Camera interface: {interface}")
                    break
        if interface is None:
            self.get_logger().info("Camera interface Not Found")
            sys.exit(0)
            
        return interface

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return

        detected_image, objectInfo = self.getObjects(frame, 0.45, 0.2)
        self.get_logger().info(f"{objectInfo}")

        # Get frame width
        frame_height, frame_width, _ = frame.shape
        mid_point = frame_width // 2
        velocity_command = Twist()

        if objectInfo:  # If objects are detected
            for obj in objectInfo:
                box, class_name, confidence = obj
                center_x = box[0] + box[2] // 2  # Calculate the center x-coordinate of the bounding box
                if center_x < mid_point:
                    side = "Left"
                    velocity_command.angular.z = -0.5  # Rotate right
                else:
                    side = "Right"
                    velocity_command.angular.z = 0.5  # Rotate left
                self.get_logger().info(f"{class_name.upper()} is on the {side} side with confidence {confidence:.2f}")

            # Publish cmd_vel only if there are detected objects
            self.cmd_vel_publisher.publish(velocity_command)

        # Resize the frame to lower the resolution
        # resized_frame = detected_image
        resized_frame = cv2.resize(detected_image, (int(320*0.7), int(205*0.7)))

        # Compress the frame
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # Adjust quality as needed
        _, compressed_data = cv2.imencode('.jpg', resized_frame, encode_param)

        # Create and publish the CompressedImage message
        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = compressed_data.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self.publisher.publish(msg)

    def getObjects(self, img, thres, nms, draw=True, objects=[]):
        classIds, confs, bbox = self.net.detect(img, confThreshold=thres, nmsThreshold=nms)
        if len(objects) == 0: 
            objects = self.classNames
        objectInfo = []
        if len(classIds) != 0:
            for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                className = self.classNames[classId - 1]
                if className in objects and confidence > self.detection_threshold:
                    objectInfo.append([box, className, confidence])
                    if draw:
                        cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                        cv2.putText(img, f"{className.upper()} {confidence:.2f}", 
                                    (box[0]+10, box[1]+30),
                                    cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
        return img, objectInfo

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()