using UnityEngine;
using ROS2;

namespace ROS2
{
    [RequireComponent(typeof(ROS2UnityComponent))]
    public class IMU2 : Sensor<sensor_msgs.msg.Imu>
    {
        private Vector3 lastPosition;
        private Quaternion lastRotation;
        private Vector3 lastVelocity;

        public Vector3 acceleration; // m/s^2
        public Vector3 angularVelocity; // rad/s
        public Quaternion orientation;

        public float publishRateHz = 30f; // 公開周波数（Hz）
        private float lastPublishTime = 0f;

        protected override void Awake()
        {
            base.Awake();
            lastPosition = transform.position;
            lastRotation = transform.rotation;
            lastVelocity = Vector3.zero;
        }

        protected override sensor_msgs.msg.Imu AcquireValue()
        {
            var msg = new sensor_msgs.msg.Imu();
            msg.Header = new std_msgs.msg.Header();
            msg.Header.Frame_id = frameID;
            ros2Node.clock.UpdateROSClockTime(msg.Header.Stamp);

            float dt = Time.deltaTime;
            Vector3 velocity = (transform.position - lastPosition) / dt;
            acceleration = (velocity - lastVelocity) / dt;
            acceleration = transform.InverseTransformDirection(acceleration);

            Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(lastRotation);
            deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f) angle -= 360f;
            angularVelocity = axis * angle * Mathf.Deg2Rad / dt;

            orientation = transform.rotation;

            lastPosition = transform.position;
            lastRotation = transform.rotation;
            lastVelocity = velocity;

            msg.Linear_acceleration.X = acceleration.x;
            msg.Linear_acceleration.Y = acceleration.y;
            msg.Linear_acceleration.Z = acceleration.z;

            msg.Angular_velocity.X = angularVelocity.x;
            msg.Angular_velocity.Y = angularVelocity.y;
            msg.Angular_velocity.Z = angularVelocity.z;

            msg.Orientation.X = orientation.x;
            msg.Orientation.Y = orientation.y;
            msg.Orientation.Z = orientation.z;
            msg.Orientation.W = orientation.w;

            return msg;
        }

        protected override bool HasNewData()
        {
            // 指定した周期でのみ新しいデータがあるとみなす
            float interval = 1.0f / publishRateHz;
            if (Time.time - lastPublishTime >= interval)
            {
                lastPublishTime = Time.time;
                return true;
            }
            return false;
        }
    }
} 