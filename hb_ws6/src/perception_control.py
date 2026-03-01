"""
* Team Id: 1060
* Author List: A. Nithish, Rohan A Khamitkar
* Filename: pose_detector.py
* Theme: Robot Localization using ArUco Markers
* Functions:
*   PoseDetector.__init__(),
*   PoseDetector.pixel_to_world(),
*   PoseDetector.image_callback(),
*   PoseDetector.publish_crate_poses(),
*   PoseDetector.publish_bot_poses(),
*   main()
* Global Variables: None
"""

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from hb_interfaces.msg import Pose2D, Poses2D


class PoseDetector(Node):
    """
    Purpose:
    ---
    ROS2 node for detecting ArUco markers from a camera feed, estimating
    their poses, converting image coordinates to world coordinates, and
    publishing bot and crate poses.
    """

    def __init__(self):
        """
        * Function Name: __init__
        * Input: None
        * Output: None
        * Logic:
        *   Initializes ROS2 node, camera parameters, ArUco detector,
        *   publishers, and subscribers.
        * Example Call: PoseDetector()
        """
        super().__init__('localization_node')

        # bridge: Converts ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # ---------- PARAMETERS ----------
        # crates_marker_length: Physical size of crate ArUco markers (meters)
        self.crates_marker_length = 0.045

        # bots_marker_length: Physical size of robot ArUco markers (meters)
        self.bots_marker_length = 0.075

        # aruco_dict_name: Dictionary used for marker detection
        self.aruco_dict_name = cv2.aruco.DICT_4X4_50

        # ---------- TOPICS ----------
        # image_sub: Subscriber for raw camera images
        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )

        # crate_poses_pub: Publishes crate poses
        self.crate_poses_pub = self.create_publisher(
            Poses2D, "/crate_pose", 10
        )

        # bot_poses_pub: Publishes robot poses
        self.bot_poses_pub = self.create_publisher(
            Poses2D, "/bot_pose", 10
        )

        # ---------- CAMERA PARAMETERS ----------
        # camera_matrix: Intrinsic camera calibration matrix
        self.camera_matrix = None

        # dist_coeffs: Lens distortion coefficients
        self.dist_coeffs = None

        # ---------- HOMOGRAPHY MATRICES ----------
        # pixel_matrix: Image-space reference points
        self.pixel_matrix = []

        # world_matrix: Corresponding real-world coordinates
        self.world_matrix = []

        # H_matrix: Homography matrix (image → world)
        self.H_matrix = None

        # ---------- ARUCO SETUP ----------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            self.aruco_dict_name
        )
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params
        )

        self.get_logger().info("PoseDetector initialized")

    def pixel_to_world(self, pixel_x, pixel_y):
        """
        * Function Name: pixel_to_world
        * Input:
        *   pixel_x -> X-coordinate in image frame
        *   pixel_y -> Y-coordinate in image frame
        * Output:
        *   (world_x, world_y) -> Tuple of world coordinates
        * Logic:
        *   Uses homography matrix to convert pixel coordinates
        *   into world coordinates.
        * Example Call: pixel_to_world(320.5, 240.2)
        """

        if self.H_matrix is None:
            self.get_logger().error("Homography matrix not computed")
            return None, None

        pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        world_point = cv2.perspectiveTransform(pixel_point, self.H_matrix)
        world_x, world_y = world_point[0][0]

        return float(world_x), float(world_y)

    def image_callback(self, msg):
        """
        * Function Name: image_callback
        * Input:
        *   msg -> ROS2 Image message
        * Output: None
        * Logic:
        *   Detects ArUco markers, computes homography using map corners,
        *   estimates orientation and position of bots and crates,
        *   and publishes their poses.
        * Example Call: Called automatically by ROS2 subscriber
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )

            # ---------- CAMERA INTRINSICS ----------
            fx, fy = 964.60617, 977.30772
            cx, cy = 908.60179, 578.86599

            self.camera_matrix = np.array(
                [[fx, 0, cx],
                 [0, fy, cy],
                 [0,  0,  1]],
                dtype=np.float32
            )

            # dist_coeffs: Assuming no lens distortion
            dist_coeffs = np.zeros(5)

            # Remove lens distortion
            undistorted_image = cv2.undistort(
                cv_image, self.camera_matrix, dist_coeffs
            )

            # Convert image to grayscale
            gray = cv2.cvtColor(
                undistorted_image, cv2.COLOR_BGR2GRAY
            )

            # Apply binary threshold for better marker detection
            _, thresh = cv2.threshold(
                gray, 125, 200, cv2.THRESH_BINARY
            )

            # Detect ArUco markers
            corners, ids, _ = self.detector.detectMarkers(thresh)

            # Mapping of marker IDs to corner indices
            end_corner = {1: 0, 3: 1, 5: 3, 7: 2}

            # Real-world coordinates of map corners (mm)
            end_corner_map = {
                1: [0, 0],
                3: [2438.4, 0],
                5: [0, 2438.4],
                7: [2438.4, 2438.4]
            }

            self.pixel_matrix = []
            self.world_matrix = []

            # Extract homography reference points
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(
                    undistorted_image, corners, ids
                )
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in end_corner:
                        self.pixel_matrix.append(
                            corners[i][0][end_corner[marker_id]]
                        )
                        self.world_matrix.append(
                            end_corner_map[marker_id]
                        )

            self.pixel_matrix = np.array(
                self.pixel_matrix, dtype=np.float32
            )
            self.world_matrix = np.array(
                self.world_matrix, dtype=np.float32
            )

            # Compute homography
            self.H_matrix, _ = cv2.findHomography(
                self.pixel_matrix,
                self.world_matrix,
                cv2.RANSAC,
                5.0
            )

            bot_poses = {}
            crate_poses = {}

            # ---------- PROCESS ALL MARKERS ----------
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in end_corner:
                    continue

                marker_corners = corners[i][0]

                # Compute marker center
                cX = np.mean(marker_corners[:, 0])
                cY = np.mean(marker_corners[:, 1])

                # Orientation using top edge
                pt1, pt2 = marker_corners[0], marker_corners[1]
                dx, dy = pt2[0] - pt1[0], pt2[1] - pt1[1]
                yaw = math.degrees(math.atan2(dy, dx)) % 360

                # Convert pixel to world coordinates
                wX, wY = self.pixel_to_world(cX, cY)

                # Perspective correction
                w_X, w_Y = 1219.2 - wX, 1219.2 - wY

                if marker_id in [0, 2, 4]:
                    shift_x = w_X * 90.0 / 2438.4
                    shift_y = w_Y * 90.0 / 2438.4
                    bot_poses[marker_id] = (
                        wX + shift_x, wY + shift_y + 10.0, yaw
                    )
                    cv2.putText(undistorted_image, f"x: {wX+shift_x:.2f}, y: {wY+shift_y+10.2:.2f} yaw: {yaw:.2f}",
                            (int(cX) + 10, int(cY) - 10), cv2.FONT_HERSHEY_SIMPLEX,0.4, (0, 255, 0), 1)
                else:
                    shift_x = w_X * 50.0 / 2438.4
                    shift_y = w_Y * 50.0 / 2438.4
                    crate_poses[marker_id] = (
                        wX + shift_x, wY + shift_y, yaw
                    )
                    cv2.putText(undistorted_image, f"x: {wX+shift_x:.2f}, y: {wY+shift_y+10.2:.2f} yaw: {yaw:.2f}",
                            (int(cX) + 10, int(cY) - 10), cv2.FONT_HERSHEY_SIMPLEX,0.4, (0, 255, 0), 1)

            self.publish_bot_poses(bot_poses)
            self.publish_crate_poses(crate_poses)

            cv2.imshow("Detected Markers", undistorted_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def publish_crate_poses(self, poses):
        """
        * Function Name: publish_crate_poses
        * Input:
        *   poses -> Dictionary of crate poses
        * Output: None
        * Logic:
        *   Converts dictionary to Poses2D message and publishes it.
        * Example Call: publish_crate_poses(crate_poses)
        """
        poses_msg = Poses2D()
        for crate_id, (x, y, yaw) in poses.items():
            pose = Pose2D()
            pose.id = int(crate_id)
            pose.x = x
            pose.y = y
            pose.w = yaw
            poses_msg.poses.append(pose)

        self.crate_poses_pub.publish(poses_msg)

    def publish_bot_poses(self, poses):
        """
        * Function Name: publish_bot_poses
        * Input:
        *   poses -> Dictionary of bot poses
        * Output: None
        * Logic:
        *   Converts dictionary to Poses2D message and publishes it.
        * Example Call: publish_bot_poses(bot_poses)
        """
        poses_msg = Poses2D()
        for bot_id, (x, y, yaw) in poses.items():
            pose = Pose2D()
            pose.id = int(bot_id)
            pose.x = x
            pose.y = y
            pose.w = yaw
            poses_msg.poses.append(pose)

        self.bot_poses_pub.publish(poses_msg)


def main(args=None):
    """
    * Function Name: main
    * Input: args -> Command line arguments
    * Output: None
    * Logic:
    *   Initializes ROS2, starts PoseDetector node, and handles shutdown.
    * Example Call: Called automatically by Python interpreter
    """
    rclpy.init(args=args)
    node = PoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()