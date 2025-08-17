import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import TransformBroadcaster


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.15,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_4X4_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="camera_position_x",
            value=0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Camera position X in map frame.",
            ),
        )

        self.declare_parameter(
            name="camera_position_y",
            value=0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Camera position Y in map frame.",
            ),
        )

        self.declare_parameter(
            name="camera_position_z",
            value=0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Camera position Z in map frame.",
            ),
        )

        self.declare_parameter(
            name="camera_orientation_yaw",
            value=0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Camera orientation yaw in radians.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        
        # camera_frameが空の場合はデフォルト値を設定
        if self.camera_frame == "":
            self.camera_frame = "camera_frame"
            self.get_logger().info("camera_frame not set, using default: camera_frame")
        
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_4X4_250):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 5)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 5)

        # Set up TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        
        # 検出率向上のためのパラメータ調整
        self.aruco_parameters.adaptiveThreshWinSizeMin = 3
        self.aruco_parameters.adaptiveThreshWinSizeMax = 23
        self.aruco_parameters.adaptiveThreshWinSizeStep = 10
        self.aruco_parameters.adaptiveThreshConstant = 7
        self.aruco_parameters.minMarkerPerimeterRate = 0.03
        self.aruco_parameters.maxMarkerPerimeterRate = 4.0
        self.aruco_parameters.polygonalApproxAccuracyRate = 0.03
        self.aruco_parameters.cornerRefinementWinSize = 5
        self.aruco_parameters.cornerRefinementMaxIterations = 30
        self.aruco_parameters.cornerRefinementMinAccuracy = 0.1
        self.aruco_parameters.markerBorderBits = 1
        self.aruco_parameters.perspectiveRemovePixelPerCell = 4
        self.aruco_parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13
        self.aruco_parameters.maxErroneousBitsInBorderRate = 0.35
        self.aruco_parameters.minOtsuStdDev = 5.0
        self.aruco_parameters.errorCorrectionRate = 0.6
        
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        
        # カメラパラメータのデバッグ情報
        self.get_logger().info(f"Camera intrinsic matrix: {self.intrinsic_mat}")
        self.get_logger().info(f"Camera distortion: {self.distortion}")
        self.get_logger().info(f"Marker size: {self.marker_size} meters")
        
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        
        # デバッグ用：画像サイズをログ出力
        self.get_logger().debug(f"Processing image: {cv_image.shape}")
        
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame


        markers.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.stamp = self.get_clock().now().to_msg()

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )
        
        # デバッグ情報を出力
        if marker_ids is not None:
            self.get_logger().info(f"Detected {len(marker_ids)} aruco markers: {marker_ids.flatten().tolist()}")
        else:
            self.get_logger().debug("No aruco markers detected")
            if rejected is not None and len(rejected) > 0:
                self.get_logger().debug(f"Rejected {len(rejected)} potential markers")
        
        if marker_ids is not None:
            if cv2.__version__ > "4.0.0":
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = -tvecs[i][0][1]  # Y軸の符号を反転
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = -quat[1]  # Y軸の符号を反転
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]


                transform = TransformStamped()
                transform.header.stamp = pose_array.header.stamp 
                transform.header.frame_id = "base_link"
                transform.child_frame_id = self.camera_frame
                
                # デバッグ情報を出力
                self.get_logger().debug(f"Broadcasting TF: map -> {self.camera_frame}")
                
                # 位置座標
                transform.transform.translation.x = tvecs[i][0][0]
                transform.transform.translation.y = -tvecs[i][0][1]  # Y軸の符号を反転
                transform.transform.translation.z = tvecs[i][0][2]
                
                # 回転
                transform.transform.rotation.x = quat[0]
                transform.transform.rotation.y = -quat[1]  # Y軸の符号を反転
                transform.transform.rotation.z = quat[2]
                transform.transform.rotation.w = quat[3]
                
                # TF変換をブロードキャスト
                self.tf_broadcaster.sendTransform(transform)

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

                

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)



def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
