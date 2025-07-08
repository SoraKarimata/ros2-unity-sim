using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;


namespace ROS2
{
    [RequireComponent(typeof(ROS2UnityComponent))]
    public class IMU : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<sensor_msgs.msg.Imu> imu_pub;

        public string frameID = "imu_link";
        public string topicName = "/imu";


        private Vector3 lastPosition;
        private Quaternion lastRotation;
        private Vector3 lastVelocity;

        public Vector3 acceleration; // m/s^2
        public Vector3 angularVelocity; // rad/s
        public Quaternion orientation; 

        // Start is called before the first frame update
        void Start()
        {
            lastPosition = transform.position;
            lastRotation = transform.rotation;
            lastVelocity = Vector3.zero;

            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Unity == null)
            {
                Debug.LogError("ROS2UnityComponent not found");
                return;
            }
            ros2Node = ros2Unity.CreateNode("imu_node");
            imu_pub = ros2Node.CreatePublisher<sensor_msgs.msg.Imu>(topicName);
        }

        // Update is called once per frame
        void Update()
        {
            if (imu_pub == null)
            {
                return;
            }   

            var msg = new sensor_msgs.msg.Imu();
            msg.Header = new std_msgs.msg.Header();
            msg.Header.Frame_id = frameID;
            ros2Node.clock.UpdateROSClockTime(msg.Header.Stamp);

            float dt = Time.deltaTime;

            Vector3 velocity = (transform.position - lastPosition) / dt;

            acceleration = (velocity - lastVelocity) / dt;
            // Convert acceleration from Unity coordinates to IMU coordinates
            acceleration = transform.InverseTransformDirection(acceleration);

            // Angular velocity
            Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(lastRotation);
            deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f) angle -= 360f;
            angularVelocity = axis * angle * Mathf.Deg2Rad / dt;

            // Orientation
            orientation = transform.rotation;

            // Next frame for calculation of acceleration and angular velocity
            lastPosition = transform.position;
            lastRotation = transform.rotation;
            lastVelocity = velocity;

            // Acceleration
            msg.Linear_acceleration.X = acceleration.x;
            msg.Linear_acceleration.Y = acceleration.y;
            msg.Linear_acceleration.Z = acceleration.z;

            // Angular velocity
            msg.Angular_velocity.X = angularVelocity.x;
            msg.Angular_velocity.Y = angularVelocity.y;
            msg.Angular_velocity.Z = angularVelocity.z;

            // Orientation
            msg.Orientation.X = orientation.x;
            msg.Orientation.Y = orientation.y;
            msg.Orientation.Z = orientation.z;
            msg.Orientation.W = orientation.w;

            imu_pub.Publish(msg);

        }

    }
}