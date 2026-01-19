#!/usr/bin/env python3

"""
This Python file runs a ROS 2 node named localization_node which publishes the position of crates and a holonomic drive robot.
This node subscribes to the following topics:
 SUBSCRIPTIONS
 /camera/image_raw
 /camera/camera_info
 /crates_pose  
 /bot_pose
"""

'''
# Team ID:          1060
# Theme:            Holo Battalion
# Author List:      A.Nithish, Rohan A Khamitkar
# Filename:         holonomic_perception.py
# Functions:        pixel_to_world, image_callback, publish_crate_poses, publish_bot_poses
# Global variables: None
'''

import math
from os import error
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from hb_interfaces.msg import Pose2D, Poses2D


class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        self.bridge = CvBridge()

        self.poses_msg = Poses2D()
        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.045  # Set marker size in meters
        self.bots_marker_length = 0.075   # Set bot marker size in meters
        self.aruco_dict_name = cv2.aruco.DICT_4X4_50  # Choose ArUco dictionary
        
        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = None  # load camera intrinsics (3x3 matrix)
        self.dist_coeffs = None    # load distortion coefficients (1x5 array)
        
        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = []  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = []  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
        
        # ---------- ARUCO SETUP ----------
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_name)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.get_logger().info('PoseDetector initialized')

    def pixel_to_world(self, pixel_x, pixel_y):
        '''
        Purpose:
        ---
        Converts pixel coordinates (x, y) into world coordinates (X, Y)
        using the homography matrix computed from reference ArUco markers.

        Input Arguments:
        ---
        pixel_x : [float] X-coordinate in image frame
        pixel_y : [float] Y-coordinate in image frame

        Returns:
        ---
        (world_x, world_y) : [tuple of float]
            The world coordinates of the given pixel position
        '''

        if not hasattr(self, "H_matrix") or self.H_matrix is None:
            print("Error: H_matrix not computed yet.")
            return None, None
        pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        world_point = cv2.perspectiveTransform(pixel_point, self.H_matrix)
        world_x, world_y = world_point[0][0]
        return float(world_x), float(world_y)
        
    def image_callback(self, msg):
        '''
        Purpose:
        ---
        Callback function for the /camera/image_raw topic.
        Processes incoming frames to detect ArUco markers, estimate their poses,
        convert to world coordinates, and publish positions of bots and crates.

        Input Arguments:
        ---
        msg : [sensor_msgs.msg.Image]
            The input image message from the ROS2 topic.

        Returns:
        ---
        None
        '''

        try:
            cv_image =self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            fx = 964.60617
            fy = 977.30772
            cx = 908.60179
            cy = 578.86599

            self.camera_matrix = np.array([
                                     [fx, 0, cx],
                                     [0, fy, cy],
                                     [0, 0, 1]
                                     ], dtype=np.float32)
            
            dist_coeffs = np.zeros(5)

            # dist_coeffs = np.array([
            #             -0.109371, 0.005711, -0.009311, 0.002713, 0.0
            #         ])
            

            undistorted_image = cv2.undistort(
                src=cv_image, 
                cameraMatrix=self.camera_matrix, 
                distCoeffs=dist_coeffs
            )
            
            grey = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

            _,thresh = cv2.threshold(grey,120,180,cv2.THRESH_BINARY)

            corners, ids, _ = self.detector.detectMarkers(thresh)

            end_corner = {
                1:0,
                3:1,
                5:3,
                7:2
            }
            
            end_corner_map = {
                1:[0, 0],
                3:[2438.4, 0],
                5:[0, 2438.4],
                7:[2438.4, 2438.4]
            }

            self.pixel_matrix = []
            self.world_matrix = []

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(undistorted_image, corners, ids)
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in [1, 3, 5, 7]:
                        self.pixel_matrix.append(corners[i][0][end_corner[marker_id]]) 
                        self.world_matrix.append(end_corner_map[marker_id])

            self.pixel_matrix = np.array(self.pixel_matrix, dtype=np.float32)
         
            self.world_matrix = np.array(self.world_matrix, dtype=np.float32)
            
            self.H_matrix, _ = cv2.findHomography(self.pixel_matrix , self.world_matrix, cv2.RANSAC, 5.0 )

            bot_poses = {}
            crate_poses = {}


            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in [1, 3, 5, 7]:
                    continue
                marker_corners = corners[i][0]
                img_points = corners[i][0].astype(np.float32)

                cX = (np.mean(marker_corners[:, 0]))
                cY = (np.mean(marker_corners[:, 1]))

                pt1 = marker_corners[0]   # top-left
                pt2 = marker_corners[1]   # top-right

                wX, wY = self.pixel_to_world(cX, cY)

                dx = pt2[0] - pt1[0]
                dy = pt2[1] - pt1[1]

                # angle in radians
                yaw = np.arctan2(dy, dx)

                yaw_rad = yaw % (2 * math.pi)
                yaw_degrees = math.degrees(yaw_rad)


                w_X, w_Y = 1219.2 - wX, 1219.2 - wY
		
                shift_x, shift_y = (0.0, 0.0)

                """
                SHIFT = (height_of_boxes/camera_height_from_ground)*coordinate
                """

                if marker_id in [0, 2, 4]:
                    shift_x, shift_y = (w_X*90.0/(2438.4), w_Y*90/(2438.4))
                    bot_poses[marker_id] = (wX + shift_x, wY + shift_y+10.0, yaw_degrees)
                    cv2.putText(undistorted_image, f"x: {wX+shift_x:.2f}, y: {wY+shift_y+10.2:.2f} yaw: {yaw_degrees:.2f}",
                            (int(cX) + 10, int(cY) - 10), cv2.FONT_HERSHEY_SIMPLEX,0.4, (0, 255, 0), 1)
                else:
                    shift_x, shift_y = (w_X*50.0/(2438.4), w_Y*50.0/(2438.4))
                    crate_poses[marker_id] = (wX + shift_x, wY + shift_y, yaw_degrees)
                    cv2.putText(undistorted_image, f"x: {wX + shift_x:.2f}, y: {wY+shift_y:.2f} yaw: {yaw_degrees:.2f}",
                            (int(cX) + 10, int(cY) - 10), cv2.FONT_HERSHEY_SIMPLEX,0.4, (0, 255, 0), 1)

            self.publish_bot_poses(bot_poses)
            self.publish_crate_poses(crate_poses)

            cv2.namedWindow("Detected Markers", cv2.WINDOW_NORMAL)
            cv2.imshow('Detected Markers', undistorted_image)
            # cv2.imshow('Detected Markers', thresh)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_crate_poses(self, poses):
        '''
        Purpose:
        ---
        Publishes crate poses to the topic `/crate_pose`
        as a Poses2D message.

        Input Arguments:
        ---
        poses : [dict]
            Dictionary containing crate marker IDs and their poses (x, y, yaw)

        Returns:
        ---
        None
        '''
        poses_msg = Poses2D()
        for crate_id, (x, y, yaw) in poses.items():
            crate_pose = Pose2D()
            crate_pose.id = int(crate_id)
            crate_pose.x = x
            crate_pose.y = y
            crate_pose.w = yaw
            poses_msg.poses.append(crate_pose)
        self.crate_poses_pub.publish(poses_msg)

    def publish_bot_poses(self, poses):
        '''
        Purpose:
        ---
        Publishes bot poses to the topic `/bot_pose`
        as a Poses2D message.

        Input Arguments:
        ---
        poses : [dict]
            Dictionary containing bot marker IDs and their poses (x, y, yaw)

        Returns:
        ---
        None
        '''
        poses_msg = Poses2D()
        for bot_id, (x, y, yaw) in poses.items():
            crate_pose = Pose2D()
            crate_pose.id = int(bot_id)
            crate_pose.x = x
            crate_pose.y = y
            crate_pose.w = yaw
            poses_msg.poses.append(crate_pose)
        self.bot_poses_pub.publish(poses_msg)


def main(args=None):
    rclpy.init(args=args)
    pose_detector = PoseDetector()
    try:
        rclpy.spin(pose_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pose_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
