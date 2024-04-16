#!/usr/bin/env python3

'''
Author : sweunwave
LiDAR - Camera Calibration based on ROS2
'''

import rclpy
import rclpy.client
from rclpy.node import Node

from cv_bridge import CvBridge 
import rclpy.node
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import PointStamped, Point32

import os
import cv2
import json
import numpy as np
from threading import Thread

# CAMERA_INTRINSIC_MATRIX = np.array([
#     [789.455662, 0.000000, 240.876141],
#     [0.000000, 791.255982, 228.114844],
#     [0.000000, 0.000000, 1.000000]
# ])

DIST_COEFFS = np.array([0.045021, -0.069708, -0.005911, -0.016257, 0.000000])

class CalibrationProcess(Node):
    def __init__(self):
        super().__init__('image_process')

        self.declare_parameter('camera_intrinsic_path', rclpy.Parameter.Type.STRING)
        json_path = self.get_parameter('camera_intrinsic_path')
        json_data = self.read_json_file(json_path.value)

        self.CAMERA_INTRINSIC_MATRIX = np.array(json_data['intrinsic_matrix'])

        # subscriber
        self.img_sub = self.create_subscription(Image, "/camera1/image_raw", self.image_callback, 10)
        self.point_sub = self.create_subscription(PointStamped, "/clicked_point", self.point_callback, 10)
        
        # publisher
        self.points_pub = self.create_publisher(PointCloud, "/calibration/clicked_points", 10)

        # camera
        self.bridge = CvBridge()
        self.selected_2d_pos = np.empty((0, 2), dtype=np.uint8)
        self.is_img = False
        self.is_thread_running = True
        
        # lidar
        self.selected_3d_points = np.empty((0, 3), dtype=np.float32)
        self.selected_3d_msg = PointCloud() 
        self.is_points = False

        # transform matrix
        self.transform_dict = {}

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        self.img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.img_time = msg.header.stamp.sec
        self.is_img =True

    def point_callback(self, msg):
        self.selected_3d_points = np.append(self.selected_3d_points, np.array([[msg.point.x, msg.point.y, msg.point.z]]), axis=0)
        print(f"3D Points : \n{self.selected_3d_points}")

        self.selected_3d_msg.header = msg.header
        self.selected_3d_msg.header.frame_id = "os_sensor"

        points = Point32()
        points.x, points.y, points.z = msg.point.x, msg.point.y, msg.point.z
        self.selected_3d_msg.points.append(points)
        self.points_pub.publish(self.selected_3d_msg)
        self.is_points =True

    def cv2_show(self):
        while rclpy.ok() and self.is_thread_running:
            if self.is_img:
                cv2.imshow("image", self.img)
                cv2.setMouseCallback("image", self.mouse_callback)
                cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(self.img, (x, y), 5, (255, 0, 0), -1)
            self.selected_2d_pos = np.append(self.selected_2d_pos, np.array([[x, y]]), axis=0)
            print(f"2D image position : \n{self.selected_2d_pos}")

        elif event == cv2.EVENT_MBUTTONDOWN:
            self.is_thread_running = False
            cv2.destroyAllWindows()

            retval, rvec, tvec = cv2.solvePnP(self.selected_3d_points, self.selected_2d_pos.astype(np.float32), self.CAMERA_INTRINSIC_MATRIX,
                                              DIST_COEFFS, rvec=None, tvec=None, useExtrinsicGuess=None, flags=None)
            
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            transform_matrix = np.hstack((rotation_matrix, tvec))
            
            print()
            print(f"rvec \n{rvec}")
            print(f"tvec \n{tvec}")
            # print(f"rotation_matrix \n{rotation_matrix}")
            # print(f"transform_matrix \n{transform_matrix}")

            self.transform_dict['rvec'] = rvec
            self.transform_dict['tvec'] = tvec

    def read_json_file(self, file_path:str) -> dict:
        with open(file_path, 'r') as json_file:
            data = json.load(json_file)
        return data

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationProcess()

    img_th = Thread(target=node.cv2_show)
    img_th.daemon = True
    img_th.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()