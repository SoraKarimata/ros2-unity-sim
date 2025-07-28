using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using System;

public class Car : MonoBehaviour {
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float maxSteeringAngle;
    public float maxRPM = 3000f; // 最大回転数（RPM）
    public float rpmLimitStart = 2500f; // 回転数制限開始点（RPM）
    public float maxBrakeTorque = 3000f; // 最大ブレーキトルク
    public float reverseRPMMultiplier = 1.5f; // 後退時の回転数制限倍率
    public float autoBrakeThreshold = 0.8f; // 自動ブレーキ開始閾値（入力の80%以上）
    public float highSpeedSteeringMultiplier = 0.7f; // 高速時のステアリング倍率
    
    // ROS2制御の設定
    public bool useROS2Control = true; // ROS2制御を使用するかどうか
    public bool fallbackToKeyboard = false; // ROS2が利用できない場合にキーボードにフォールバックするか
    public float ros2Timeout = 0.01f; // ROS2データのタイムアウト時間（秒）- 100msに延長

    // ROS2関連の変数（rover_control.csから移植）
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<geometry_msgs.msg.Twist> cmd_vel_sub;
    
    // 受信した制御データ
    [HideInInspector]
    public float linearVelocity = 0f;
    [HideInInspector]
    public float angularVelocity = 0f;
    
    // 制御データのスケーリング係数
    public float linearScale = 1.0f;
    public float angularScale = 1.0f;
    
    private Rigidbody carRigidbody;
    private float lastROS2DataTime = 0f;
    private bool ros2DataReceived = false; // ROS2データ受信フラグ
    private float lastSteering = 0f; // 前回のsteering値を保持
    private float lastMotor = 0f; // 前回のmotor値を保持
    
    void Start() {
        carRigidbody = GetComponent<Rigidbody>();
        
        // ROS2コンポーネントを取得
        if (useROS2Control)
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Unity == null)
            {
                Debug.LogWarning("ROS2UnityComponent not found. Falling back to keyboard control.");
                useROS2Control = false;
            }
        }
    }
    
    void Update()
    {
        // ROS2ノードとサブスクリプションの初期化
        if (useROS2Control && ros2Node == null && ros2Unity != null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("rover_control_sub");
            cmd_vel_sub = ros2Node.CreateSubscription<geometry_msgs.msg.Twist>(
                "cmd_vel", OnCmdVelReceived);
                
            Debug.Log("ROS2 Rover Control Subscriber initialized");
        }
    }
    
    // cmd_velメッセージを受信した時のコールバック（rover_control.csから移植）
    private void OnCmdVelReceived(geometry_msgs.msg.Twist msg)
    {
        // 線形速度（前進・後退）
        linearVelocity = (float)(msg.Linear.X * linearScale);
        
        // 角速度（旋回）
        angularVelocity = (float)(-msg.Angular.Z * angularScale);
        // ROS2データ受信フラグを設定
        ros2DataReceived = true;
    }
    
    public void FixedUpdate() {
        // ROS2データ受信フラグをチェックして時間を更新（メインスレッドで実行）
        if (ros2DataReceived)
        {
            lastROS2DataTime = Time.time;
            ros2DataReceived = false;
        }
        
        float motor = lastMotor;
        float steering = lastSteering; // 前回の値を初期値として使用

        float brake = 0f;
        float break_point_vel = 0.05f; // break poinr from twist dara m/s
        
        // 入力ソースを決定
        bool useROS2 = useROS2Control && 
                      (Time.time - lastROS2DataTime) < ros2Timeout;
        
        if (useROS2)
        {
            // ROS2からの制御データを使用（直接内部変数から取得）
            float linearInput = GetLinearInput();
            float angularInput = GetAngularInput();

            // デッドゾーンを小さくして、スムーズな制御を実現
            if(Mathf.Abs(angularInput) > 0.3 && Mathf.Abs(linearInput) < 0.1){
                linearInput = 1.0f;
            }

            if(Mathf.Abs(angularInput) < 0.2){
                angularInput = 0.0f;
            }
            
            motor = maxMotorTorque * linearInput * 0.2f;
            steering = maxSteeringAngle * angularInput;
            
            // 現在のsteering値を保存
            lastSteering = steering;
            lastMotor = motor;

            
            if ( break_point_vel > Mathf.Abs(linearInput))
            {
                brake = maxBrakeTorque * 50 * 0.3f;
                // Debug.Log($"Brake applied: {brake} (linearInput: {linearInput})");
            }
            
            // デバッグログを追加
            Debug.Log($"Calculated a: {angularVelocity} , m: {linearVelocity}");
        }
        else if (fallbackToKeyboard)
        {
            // キーボード入力にフォールバック
            motor = maxMotorTorque * Input.GetAxis("Vertical");
            steering = maxSteeringAngle * Input.GetAxis("Horizontal");
            brake = maxBrakeTorque * Input.GetAxis("Jump"); // スペースキーでブレーキ
        }
        
        // 車の速度を取得
        float currentSpeed = carRigidbody.velocity.magnitude;
        float maxSpeed = 30f; // 最大速度（調整可能）
        float speedRatio = currentSpeed / maxSpeed;
        
        
        foreach (AxleInfo axleInfo in axleInfos) {
            if (axleInfo.steering) {
                axleInfo.leftFrontWheel.steerAngle = steering;
                axleInfo.rightFrontWheel.steerAngle = steering;
                axleInfo.leftBackWheel.steerAngle = -steering;
                axleInfo.rightBackWheel.steerAngle = -steering;
            }
            if (axleInfo.motor) {
                // 各ホイールの回転数をチェックして制限を適用
                float leftFrontRPM = axleInfo.leftFrontWheel.rpm;
                float rightFrontRPM = axleInfo.rightFrontWheel.rpm;
                float leftBackRPM = axleInfo.leftBackWheel.rpm;
                float rightBackRPM = axleInfo.rightBackWheel.rpm;
                
                // 回転数に応じてモータートルクを調整
                float leftFrontMotor = CalculateMotorTorque(motor, leftFrontRPM);
                float rightFrontMotor = CalculateMotorTorque(motor, rightFrontRPM);
                float leftBackMotor = CalculateMotorTorque(motor, leftBackRPM);
                float rightBackMotor = CalculateMotorTorque(motor, rightBackRPM);
                
                axleInfo.leftFrontWheel.motorTorque = leftFrontMotor;
                axleInfo.rightFrontWheel.motorTorque = rightFrontMotor;
                axleInfo.leftBackWheel.motorTorque = leftBackMotor;
                axleInfo.rightBackWheel.motorTorque = rightBackMotor;
            }
            
            // // ブレーキを適用
            axleInfo.leftFrontWheel.brakeTorque = brake;
            axleInfo.rightFrontWheel.brakeTorque = brake;
            axleInfo.leftBackWheel.brakeTorque = brake;
            axleInfo.rightBackWheel.brakeTorque = brake;

            // タイヤの見た目をコライダーに合わせて動かす
            UpdateWheelVisual(axleInfo.leftFrontWheel, axleInfo.leftFrontWheelModel);
            UpdateWheelVisual(axleInfo.rightFrontWheel, axleInfo.rightFrontWheelModel);
            UpdateWheelVisual(axleInfo.leftBackWheel, axleInfo.leftBackWheelModel);
            UpdateWheelVisual(axleInfo.rightBackWheel, axleInfo.rightBackWheelModel);
            
            // ステアリング角度と同じ角度で別の3Dモデルも回転
            UpdateSteeringModel(axleInfo.leftFrontSteeringModel, steering);
            UpdateSteeringModel(axleInfo.rightFrontSteeringModel, steering);
            UpdateSteeringModel(axleInfo.leftBackSteeringModel, -steering);
            UpdateSteeringModel(axleInfo.rightBackSteeringModel, -steering);
            // Debug.Log($"ROS2 Steering: {steering} , Motor: {motor}");
        }
    }
    
    // 現在の制御値を取得するメソッド（rover_control.csから移植）
    public float GetLinearInput()
    {
        return (float)linearVelocity;
    }
    
    public float GetAngularInput()
    {
        return (float)angularVelocity;
    }
    
    // 制御データをリセット（rover_control.csから移植）
    public void ResetControlData()
    {
        linearVelocity = 0f;
        angularVelocity = 0f;
    }

    // 回転数に応じてモータートルクを計算する関数
    private float CalculateMotorTorque(float baseMotor, float rpm) {
        float absRPM = Mathf.Abs(rpm);
        
        // 後退時（負のモータートルク）は回転数制限を緩くする
        float currentMaxRPM = maxRPM;
        float currentRpmLimitStart = rpmLimitStart;
        
        if (baseMotor < 0) {
            currentMaxRPM *= reverseRPMMultiplier;
            currentRpmLimitStart *= reverseRPMMultiplier;
        }
        
        // 最大回転数を超えている場合は0
        if (absRPM >= currentMaxRPM) {
            return 0f;
        }
        
        // 回転数制限開始点を超えている場合は徐々に減らす
        if (absRPM > currentRpmLimitStart) {
            float t = (absRPM - currentRpmLimitStart) / (currentMaxRPM - currentRpmLimitStart);
            float result = baseMotor * (1f - t);
            Debug.Log($"RPM limit active: t={t}, result={result}");
            return result;
        }
        
        // 通常のモータートルク
        return baseMotor;
    }

    // タイヤの見た目をコライダーに合わせて動かす関数
    private void UpdateWheelVisual(WheelCollider collider, Transform model) {
        if (model == null) return;
        Vector3 pos;
        Quaternion rot;
        collider.GetWorldPose(out pos, out rot);
        model.position = pos;
        model.rotation = rot;
    }
    
    // ステアリング角度と同じ角度で別の3Dモデルを回転させる関数
    private void UpdateSteeringModel(Transform steeringModel, float steeringAngle) {
        if (steeringModel == null) return;
        
        // Y軸周りの回転を適用（ステアリング角度）
        steeringModel.localRotation = Quaternion.Euler(0f, steeringAngle, 0f);
    }
}
[System.Serializable]
public class AxleInfo {
    public WheelCollider leftFrontWheel;
    public WheelCollider rightFrontWheel;
    public WheelCollider leftBackWheel;
    public WheelCollider rightBackWheel;
    public Transform leftFrontWheelModel;
    public Transform rightFrontWheelModel;
    public Transform leftBackWheelModel;
    public Transform rightBackWheelModel;
    public Transform leftFrontSteeringModel; // 左前ステアリングモデル
    public Transform rightFrontSteeringModel; // 右前ステアリングモデル
    public Transform leftBackSteeringModel; // 左後ステアリングモデル
    public Transform rightBackSteeringModel; // 右後ステアリングモデル
    public bool motor; //駆動輪か?
    public bool steering; //ハンドル操作をしたときに角度が変わるか？
}