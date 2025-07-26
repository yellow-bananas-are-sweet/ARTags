import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArTagDetector(Node):
    def __init__(self):
        super().__init__('artag_detector_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)
        self.image_pub = self.create_publisher(Image, 'artag/image_annotated', 10)
        self.pose_pub = self.create_publisher(PoseArray, 'artag/poses', 10)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.marker_length = 0.1
        self.camera_matrix = np.array([[800, 0, 320],
                                       [0, 800, 240],
                                       [0,   0,   1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict)

        pose_array = PoseArray()
        pose_array.header = msg.header

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for rvec, tvec in zip(rvecs, tvecs):
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = tvec[0]
                pose.orientation.x, pose.orientation.y, pose.orientation.z = rvec[0]
                pose_array.poses.append(pose)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='rgb8'))
        self.pose_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = ArTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()