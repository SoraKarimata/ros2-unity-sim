using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
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
    public bool fallbackToKeyboard = true; // ROS2が利用できない場合にキーボードにフォールバックするか
    public float ros2Timeout = 0.01f; // ROS2データのタイムアウト時間（秒）

    
    private Rigidbody carRigidbody;
    private ROS2RoverControlSub ros2Controller;
    private float lastROS2DataTime = 0f;
    private bool ros2DataReceived = false; // ROS2データ受信フラグ
    
    void Start() {
        carRigidbody = GetComponent<Rigidbody>();
        
        // ROS2コントローラーを探す
        if (useROS2Control)
        {
            ros2Controller = FindObjectOfType<ROS2RoverControlSub>();
            if (ros2Controller == null)
            {
                Debug.LogWarning("ROS2RoverControlSub not found. Falling back to keyboard control.");
                useROS2Control = false;
            }
        }
    }
    
    public void FixedUpdate() {
        // ROS2データ受信フラグをチェックして時間を更新（メインスレッドで実行）
        if (ros2DataReceived)
        {
            lastROS2DataTime = Time.time;
            ros2DataReceived = false;
        }
        
        float motor = 0f;
        float steering = 0f;
        float brake = 0f;
        float break_point_vel = 0.1f; // break poinr from twist dara m/s
        
        // 入力ソースを決定
        bool useROS2 = useROS2Control && ros2Controller != null && 
                      (Time.time - lastROS2DataTime) < ros2Timeout;
        
        if (useROS2)
        {
            // ROS2からの制御データを使用
            motor = maxMotorTorque * ros2Controller.GetLinearInput();
            steering = maxSteeringAngle * ros2Controller.GetAngularInput();
            
            float ROS2motorInput = ros2Controller.GetLinearInput();
            if ( break_point_vel > Mathf.Abs(ROS2motorInput))
            {
                brake = maxBrakeTorque * 50 * 0.3f;
            }
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
        
        // 高速時のステアリング調整
        if (speedRatio > 0.7f) {
            steering *= highSpeedSteeringMultiplier;
        }
        
        // 方向転換時の自動ブレーキ（キーボード入力時）
        if (!useROS2 && fallbackToKeyboard)
        {
            float steeringInput = Mathf.Abs(Input.GetAxis("Horizontal"));
            if (steeringInput > autoBrakeThreshold && speedRatio > 0.5f) {
                brake = Mathf.Max(brake, maxBrakeTorque * steeringInput * 0.3f);
            }
        }
        
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
            
            // ブレーキを適用
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
        }
    }
    
    // ROS2データを受信した時に呼び出される（rover_control.csから呼び出し）
    public void OnROS2DataReceived()
    {
        ros2DataReceived = true;
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
            return baseMotor * (1f - t);
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