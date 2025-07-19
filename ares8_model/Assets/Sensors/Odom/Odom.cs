using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace ROS2
{
    [RequireComponent(typeof(ROS2UnityComponent))]
    public class Odom : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<nav_msgs.msg.Odometry> odom_pub;

        public string frameID = "odom";
        public string childFrameID = "base_link";
        public string topicName = "/odom";

        private Vector3 lastPosition;
        private Quaternion lastRotation;
        private Vector3 lastVelocity;
        private Vector3 lastAngularVelocity;

        public Vector3 position; // 位置 (m)
        public Vector3 linearVelocity; // 線形速度 (m/s)
        public Vector3 angularVelocity; // 角速度 (rad/s)
        public Quaternion orientation; // 姿勢

        // Start is called before the first frame update
        void Start()
        {
            lastPosition = transform.position;
            lastRotation = transform.rotation;
            lastVelocity = Vector3.zero;
            lastAngularVelocity = Vector3.zero;

            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Unity == null)
            {
                Debug.LogError("ROS2UnityComponent not found");
                return;
            }
            ros2Node = ros2Unity.CreateNode("odom_node");
            odom_pub = ros2Node.CreatePublisher<nav_msgs.msg.Odometry>(topicName);
        }

        // Update is called once per frame
        void Update()
        {
            if (odom_pub == null)
            {
                return;
            }

            var msg = new nav_msgs.msg.Odometry();
            msg.Header = new std_msgs.msg.Header();
            msg.Header.Frame_id = frameID;
            ros2Node.clock.UpdateROSClockTime(msg.Header.Stamp);

            float dt = Time.deltaTime;

            // 位置
            position = transform.position;
            // Unity座標系からROS座標系に変換
            position = position.Unity2Ros();

            // 線形速度
            linearVelocity = (transform.position - lastPosition) / dt;
            // Unity座標系からROS座標系に変換
            linearVelocity = linearVelocity.Unity2Ros();

            // 角速度
            Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(lastRotation);
            deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f) angle -= 360f;
            angularVelocity = axis * angle * Mathf.Deg2Rad / dt;
            // Unity座標系からROS座標系に変換
            angularVelocity = angularVelocity.Unity2Ros();

            // 姿勢
            orientation = transform.rotation;
            // Unity座標系からROS座標系に変換
            orientation = orientation.Unity2Ros();

            // 次のフレームの計算のために現在の値を保存
            lastPosition = transform.position;
            lastRotation = transform.rotation;
            lastVelocity = linearVelocity;
            lastAngularVelocity = angularVelocity;

            // 位置と姿勢を設定
            msg.Pose.Pose.Position.X = position.x;
            msg.Pose.Pose.Position.Y = position.y;
            msg.Pose.Pose.Position.Z = position.z;

            msg.Pose.Pose.Orientation.X = orientation.x;
            msg.Pose.Pose.Orientation.Y = orientation.y;
            msg.Pose.Pose.Orientation.Z = orientation.z;
            msg.Pose.Pose.Orientation.W = orientation.w;

            // 速度を設定
            msg.Twist.Twist.Linear.X = linearVelocity.x;
            msg.Twist.Twist.Linear.Y = linearVelocity.y;
            msg.Twist.Twist.Linear.Z = linearVelocity.z;

            msg.Twist.Twist.Angular.X = angularVelocity.x;
            msg.Twist.Twist.Angular.Y = angularVelocity.y;
            msg.Twist.Twist.Angular.Z = angularVelocity.z;

            // 子フレームIDを設定
            msg.Child_frame_id = childFrameID;

            // 共分散行列を設定（簡易的な設定）
            // 位置の共分散（6x6行列）
            for (int i = 0; i < 36; i++)
            {
                msg.Pose.Covariance[i] = 0.0;
            }
            // 対角成分のみ設定（位置と姿勢の不確実性）
            msg.Pose.Covariance[0] = 0.01;  // x位置の分散
            msg.Pose.Covariance[7] = 0.01;  // y位置の分散
            msg.Pose.Covariance[14] = 0.01; // z位置の分散
            msg.Pose.Covariance[21] = 0.01; // x姿勢の分散
            msg.Pose.Covariance[28] = 0.01; // y姿勢の分散
            msg.Pose.Covariance[35] = 0.01; // z姿勢の分散

            // 速度の共分散（6x6行列）
            for (int i = 0; i < 36; i++)
            {
                msg.Twist.Covariance[i] = 0.0;
            }
            // 対角成分のみ設定（線形速度と角速度の不確実性）
            msg.Twist.Covariance[0] = 0.1;  // x線形速度の分散
            msg.Twist.Covariance[7] = 0.1;  // y線形速度の分散
            msg.Twist.Covariance[14] = 0.1; // z線形速度の分散
            msg.Twist.Covariance[21] = 0.1; // x角速度の分散
            msg.Twist.Covariance[28] = 0.1; // y角速度の分散
            msg.Twist.Covariance[35] = 0.1; // z角速度の分散

            odom_pub.Publish(msg);
        }
    }
}
