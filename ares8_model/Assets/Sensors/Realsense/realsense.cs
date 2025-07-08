using UnityEngine;

public class DepthDistanceLogger : MonoBehaviour
{
    [Header("カメラ設定")]
    public Camera depthCamera;
    public LayerMask targetLayer = -1; // すべてのレイヤーを対象
    
    [Header("距離測定設定")]
    public float maxDistance = 10f; // 最大測定距離
    public Transform raycastOrigin; // レイキャストの開始位置（nullの場合はカメラの位置）
    
    private bool isInitialized = false;
    
    void Start()
    {
        InitializeCamera();
    }
    
    void InitializeCamera()
    {
        try
        {
            // カメラが設定されていない場合はメインカメラを探す
            if (depthCamera == null)
            {
                depthCamera = Camera.main;
                
                // メインカメラが見つからない場合は、シーン内の最初のカメラを探す
                if (depthCamera == null)
                {
                    Camera[] cameras = FindObjectsOfType<Camera>();
                    if (cameras.Length > 0)
                    {
                        depthCamera = cameras[0];
                        Debug.Log($"メインカメラが見つからないため、最初のカメラを使用します: {depthCamera.name}");
                    }
                }
            }
            
            if (depthCamera == null)
            {
                Debug.LogError("シーン内にカメラが見つかりません。カメラを追加するか、Inspectorでカメラを指定してください。");
                return;
            }
            
            isInitialized = true;
            Debug.Log($"深度測定システムが初期化されました。使用カメラ: {depthCamera.name}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"カメラの初期化中にエラーが発生しました: {e.Message}");
        }
    }

    void Update()
    {
        if (!isInitialized || depthCamera == null) 
        {
            // 初期化されていない場合は再試行
            if (!isInitialized)
            {
                InitializeCamera();
            }
            return;
        }

        try
        {
            // レイキャストの開始位置を決定
            Vector3 origin = raycastOrigin != null ? raycastOrigin.position : depthCamera.transform.position;
            
            // カメラの前方方向を取得
            Vector3 direction = depthCamera.transform.forward;
            
            // レイキャストを実行して深度情報を取得
            RaycastHit hit;
            if (Physics.Raycast(origin, direction, out hit, maxDistance, targetLayer))
            {
                // ヒットしたオブジェクトとの距離を計算
                float distance = hit.distance;
                
                // 距離をログ出力
                Debug.Log($"目の前の対象との距離: {distance:F3} メートル");
                Debug.Log($"対象オブジェクト: {hit.collider.gameObject.name}");
                Debug.Log($"ヒット位置: {hit.point}");
                
                // 距離が近すぎる場合の警告
                if (distance < 0.5f)
                {
                    Debug.LogWarning($"対象が近すぎます！距離: {distance:F3} メートル");
                }
                
                // ヒットしたオブジェクトの情報を詳細に表示
                Debug.Log($"オブジェクトのレイヤー: {LayerMask.LayerToName(hit.collider.gameObject.layer)}");
                Debug.Log($"オブジェクトのタグ: {hit.collider.gameObject.tag}");
            }
            else
            {
                Debug.Log($"前方 {maxDistance} メートル以内に対象が見つかりません");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"深度測定中にエラーが発生しました: {e.Message}");
        }
    }

    // シーンビューでレイキャストの可視化（デバッグ用）
    void OnDrawGizmos()
    {
        if (depthCamera == null) return;
        
        try
        {
            Vector3 origin = raycastOrigin != null ? raycastOrigin.position : depthCamera.transform.position;
            Vector3 direction = depthCamera.transform.forward;
            
            // レイキャストの線を描画
            Gizmos.color = Color.red;
            Gizmos.DrawRay(origin, direction * maxDistance);
            
            // レイキャストの開始位置を球で表示
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(origin, 0.1f);
        }
        catch (System.Exception e)
        {
            // Gizmosでのエラーは無視
        }
    }
}