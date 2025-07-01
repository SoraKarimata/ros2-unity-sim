using System;
using UnityEngine;

namespace ROS2
{

    /// <summary>
    /// ROS2から車両制御コマンドを受信してCar.csに渡すクラス
    /// </summary>
    public class ROS2RoverControlSub : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private ISubscription<geometry_msgs.msg.Twist> cmd_vel_sub;
        
        // Car.csへの参照
        public Car carController;
        
        // 受信した制御データ
        [HideInInspector]
        public float linearVelocity = 0f;
        [HideInInspector]
        public float angularVelocity = 0f;
        
        // 制御データのスケーリング係数
        public float linearScale = 1.0f;
        public float angularScale = 1.0f;

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            
            // Car.csが見つからない場合は自動で探す
            if (carController == null)
            {
                carController = FindObjectOfType<Car>();
            }
        }

        void Update()
        {
            if (ros2Node == null && ros2Unity.Ok())
            {
                ros2Node = ros2Unity.CreateNode("rover_control_sub");
                cmd_vel_sub = ros2Node.CreateSubscription<geometry_msgs.msg.Twist>(
                    "cmd_vel", OnCmdVelReceived);
                    
                Debug.Log("ROS2 Rover Control Subscriber initialized");
            }
        }
        
        // cmd_velメッセージを受信した時のコールバック
        private void OnCmdVelReceived(geometry_msgs.msg.Twist msg)
        {
            // 線形速度（前進・後退）
            linearVelocity = (float)(msg.Linear.X * linearScale);
            
            // 角速度（旋回）
            angularVelocity = (float)(-msg.Angular.Z * angularScale);
            
            Debug.Log($"Received cmd_vel - Linear: {linearVelocity}, Angular: {angularVelocity}");
            
            // Car.csにROS2データ受信を通知
            if (carController != null)
            {
                carController.OnROS2DataReceived();
            }
        }
        
        // Car.csから呼び出されるメソッド：現在の制御値を取得
        public float GetLinearInput()
        {
            return Mathf.Clamp((float)linearVelocity, -1f, 1f);
        }
        
        public float GetAngularInput()
        {
            return Mathf.Clamp((float)angularVelocity, -1f, 1f);
        }
        
        // 制御データをリセット
        public void ResetControlData()
        {
            linearVelocity = 0f;
            angularVelocity = 0f;
        }
    }

}  // namespace ROS2