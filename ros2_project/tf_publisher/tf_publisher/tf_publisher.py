import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TFPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.odo_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.latest_imu_data = None
        self.latest_odom_data = None
        self.tf_broadcaster = TransformBroadcaster(self)

    def imu_callback(self, msg):
        self.latest_imu_data = msg
        self.publish_tf()

    def odom_callback(self, msg):
        self.latest_odom_data = msg
        self.publish_tf()
        
    def publish_tf(self):
        if self.latest_imu_data is None or self.latest_odom_data is None:
            return

        # static tf base_link → gps_link
        t_gps = TransformStamped()
        t_gps.header.stamp = self.get_clock().now().to_msg()
        t_gps.header.frame_id = 'base_link'
        t_gps.child_frame_id = 'gps_link'
        t_gps.transform.translation.x = 0.0
        t_gps.transform.translation.y = 0.0
        t_gps.transform.translation.z = 0.0
        t_gps.transform.rotation.x = 0.0
        t_gps.transform.rotation.y = 0.0
        t_gps.transform.rotation.z = 0.0
        t_gps.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_gps)

        # static tf map → odom
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = self.get_clock().now().to_msg()
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation.x = 0.0
        t_map_odom.transform.rotation.y = 0.0
        t_map_odom.transform.rotation.z = 0.0
        t_map_odom.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map_odom)

        # static tf odom → base_link
        t_odom_baselink = TransformStamped()
        t_odom_baselink.header.stamp = self.get_clock().now().to_msg()
        t_odom_baselink.header.frame_id = 'odom'
        t_odom_baselink.child_frame_id = 'base_link'
        t_odom_baselink.transform.translation.x = self.latest_odom_data.pose.pose.position.x
        t_odom_baselink.transform.translation.y = self.latest_odom_data.pose.pose.position.y
        t_odom_baselink.transform.translation.z = self.latest_odom_data.pose.pose.position.z

        # IMUデータから回転情報を取得
        t_odom_baselink.transform.rotation.x = self.latest_imu_data.orientation.x
        t_odom_baselink.transform.rotation.y = self.latest_imu_data.orientation.y
        t_odom_baselink.transform.rotation.z = self.latest_imu_data.orientation.z
        t_odom_baselink.transform.rotation.w = self.latest_imu_data.orientation.w

        self.tf_broadcaster.sendTransform(t_odom_baselink)

    def timer_callback(self):
        if self.latest_imu_data is not None:
            self.publish_tf()


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
