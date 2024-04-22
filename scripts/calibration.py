#!/usr/bin/env python3

'''
Author : sweunwave
LiDAR - Camera Calibration based on ROS2
'''

import rclpy
import rclpy.client
import rclpy.logging
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

class CalibrationProcess(Node):
    def __init__(self):
        super().__init__('image_process')

        self.declare_parameter('camera_params_path', rclpy.Parameter.Type.STRING)
        json_path = self.get_parameter('camera_params_path')
        json_data = self.read_json_file(json_path.value)

        self.CAMERA_INTRINSIC_MATRIX = np.array(json_data['intrinsic_matrix'])
        self.DIST_COEFFS = np.array(json_data['dist_coeffs'])

        # subscriber
        self.img_sub = self.create_subscription(Image, "/camera1/image_raw", self.image_callback, 10)
        self.point_sub = self.create_subscription(PointStamped, "/clicked_point", self.point_callback, 10)
        
        # publisher
        self.points_pub = self.create_publisher(PointCloud, "/calibration/clicked_points", 10)

        # camera
        self.bridge = CvBridge()
        self.selected_2d_pos = np.empty((0, 2), dtype=np.float32)
        self.total_selected_2d_pos = np.empty((0, 2), dtype=np.float32)
        self.is_img = False
        self.is_thread_running = True
        self.pre_img = None
        self.selection_turn_3d_points = False # 3d lidar point 를 선택 할 차례인지 아닌지 통제

        # lidar
        self.selected_3d_points = np.empty((0, 3), dtype=np.float32)
        self.total_selected_3d_points = np.empty((0, 3), dtype=np.float32)
        self.selected_3d_msg = PointCloud() 
        self.is_points = False

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        self.img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.origin_img = self.img.copy()
        self.is_img =True

    def point_callback(self, msg):
        self.selected_3d_msg.header = msg.header
        self.selected_3d_msg.header.frame_id = "os_sensor"

        points = Point32()
        points.x, points.y, points.z = msg.point.x, msg.point.y, msg.point.z

        if len(self.selected_2d_pos) > 0 and self.selection_turn_3d_points:
            self.selected_3d_points = np.append(self.selected_3d_points, np.array([[msg.point.x, msg.point.y, msg.point.z]]), axis=0)
            self.selected_3d_msg.points.append(points)
            self.points_pub.publish(self.selected_3d_msg)
            print(f"3D Points : \n{self.selected_3d_points}")
            self.selection_turn_3d_points = False
        else:
            print("[Error] You must select an image point first on OpenCV")

    def cv2_show(self):
        while rclpy.ok() and self.is_thread_running:
            if self.is_img:
                cv2.imshow("image", self.img)
                cv2.setMouseCallback("image", self.mouse_callback)
                cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and flags != cv2.EVENT_FLAG_CTRLKEY+1:
            if self.selection_turn_3d_points == False:
                self.selected_2d_pos = np.append(self.selected_2d_pos, np.array([[x, y]]), axis=0)
                print(f"2D image position : \n{self.selected_2d_pos}")
                self.selection_turn_3d_points = True
            else:
                print("You must select an 3D lidar point on RVIZ")

            for point in self.selected_2d_pos:
                    cv2.circle(self.img, (int(point[0]), int(point[1])), 5, (255, 0, 0), -1)

        elif event == cv2.EVENT_LBUTTONDOWN and flags == cv2.EVENT_FLAG_CTRLKEY+1:
            if len(self.selected_2d_pos) > 0:
                if len(self.selected_2d_pos) == len(self.selected_3d_points):
                    self.selected_2d_pos = np.delete(self.selected_2d_pos, -1, axis=0)
                    self.selected_3d_points = np.delete(self.selected_3d_points, -1, axis=0)

                    self.selected_3d_msg.points.pop()
                    self.points_pub.publish(self.selected_3d_msg)
                    self.selection_turn_3d_points = False
                else:
                    self.selected_2d_pos = np.delete(self.selected_2d_pos, -1, axis=0)
                    self.selection_turn_3d_points = False
                    
                print(f"2D image position 2: \n{self.selected_2d_pos}")
                print(f"3D Points : \n{self.selected_3d_points}")

                self.img = self.origin_img.copy()
                for point in self.selected_2d_pos:
                    cv2.circle(self.img, (int(point[0]), int(point[1])), 5, (255, 0, 0), -1)
                cv2.imshow("image", self.img)
            else:
                print("any points did not selected")
                
        elif event == cv2.EVENT_MBUTTONDOWN and flags != cv2.EVENT_FLAG_CTRLKEY+4:
            # merge to total
            self.total_selected_3d_points = np.append(self.total_selected_3d_points, self.selected_3d_points, axis=0)
            self.total_selected_2d_pos = np.append(self.total_selected_2d_pos, self.selected_2d_pos, axis=0)

            # re-init
            self.selected_2d_pos = np.empty((0, 2), dtype=np.float32)
            self.selected_3d_points = np.empty((0, 3), dtype=np.float32)
            
            self.selected_3d_msg.points.clear()
            self.points_pub.publish(self.selected_3d_msg)

            self.img = self.origin_img.copy()
            for point in self.selected_2d_pos:
                cv2.circle(self.img, (int(point[0]), int(point[1])), 5, (255, 0, 0), -1)
            cv2.imshow("image", self.img)

            print(f"num of total 3d points : {len(self.total_selected_3d_points)}")
            print(f"num of total 2d pos    : {len(self.total_selected_2d_pos)}")

        elif event == cv2.EVENT_MBUTTONDOWN and flags == cv2.EVENT_FLAG_CTRLKEY+4:
            if len(self.total_selected_3d_points) >= 6:
                self.is_thread_running = False

                retval, rvec, tvec = cv2.solvePnP(self.total_selected_3d_points, self.total_selected_2d_pos, self.CAMERA_INTRINSIC_MATRIX,
                                                  self.DIST_COEFFS, rvec=None, tvec=None, useExtrinsicGuess=None, flags=None)

                # rotation_matrix, _ = cv2.Rodrigues(rvec)
                # transform_matrix = np.hstack((rotation_matrix, tvec))

                retval, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(self.total_selected_3d_points, 
                    self.total_selected_2d_pos, self.CAMERA_INTRINSIC_MATRIX, self.DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE)

                print()
                print(f"rvec solveRansac\n{rotation_vector}")
                print(f"rvec solve \n{rvec}")

                print(f"tvec solveRansac\n{translation_vector}")
                print(f"tvec solve \n{tvec}")
            else:
                print("Must select at least 6 points")

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