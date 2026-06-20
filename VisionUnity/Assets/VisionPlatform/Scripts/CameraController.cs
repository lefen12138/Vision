/**
 * CameraController.cs —— Unity 虚拟相机参数控制脚本
 *
 * 功能：
 *   1. 暴露焦距、传感器尺寸等相机参数为 Inspector 可调节字段
 *   2. 参数变化时，同步更新 Unity Camera 组件和 C++ DLL 标定参数
 *   3. 通过 DLL P/Invoke 实现虚拟场景与实际算法的参数联动
 *
 * 使用方式：
 *   1. 将此脚本挂载到 Unity 场景中的 Camera 游戏对象
 *   2. 在 Inspector 面板中调整焦距、传感器尺寸等参数
 *   3. 确保 VisionCore.dll 已放置在 Assets/Plugins 目录下
 *
 * 依赖：
 *   - UnityEngine 原生 API（Camera, Mathf, Debug 等）
 *   - VisionCore.dll（C++ 核心算法库）
 *   - （可选）OpenCVForUnity 插件（用于虚拟场景图像采集后送入 DLL）
 */

using System;
using System.Runtime.InteropServices;
using System.IO;
using UnityEngine;

namespace VisionPlatform
{
    // ============================================================
    // 结果结构体（C# 端，与 C++ ResultStruct 内存布局一致）
    // ============================================================
    [StructLayout(LayoutKind.Sequential, Pack = 1, CharSet = CharSet.Ansi)]
    public struct ResultStruct
    {
        public double CenterX;
        public double CenterY;
        public double WidthMM;
        public double HeightMM;
        public double NormalLengthMM;
        public double RotationAngle;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)]
        public string ClassName;
    }

    // ============================================================
    // VisionCore DLL P/Invoke（Unity 版本）
    //
    // 注意：Unity 中 DLL 应放置于 Assets/Plugins/x86_64/ 目录
    //       文件名不含路径，Unity 编辑器自动查找
    // ============================================================
    public static class VisionBridgeUnity
    {
        private const string DLL_NAME = "VisionCore";   // Unity 会自动加 .dll 后缀

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void ProcessFrame(
            IntPtr data, int width, int height, int step,
            out ResultStruct outResult);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl,
                   CharSet = CharSet.Ansi)]
        public static extern void SetCalibrationParams(string xmlPath);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetWaveletScales(int scales);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl,
                   CharSet = CharSet.Ansi)]
        public static extern void SetHuTemplatePath(string csvPath);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetHuThreshold(double threshold);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr GetVersion();

        public static string GetVersionString()
        {
            try { return Marshal.PtrToStringAnsi(GetVersion()) ?? ""; }
            catch { return "DLL Unavailable"; }
        }
    }

    // ============================================================
    // CameraController —— 主控制脚本
    // ============================================================
    public class CameraController : MonoBehaviour
    {
        // ============================================================
        // Inspector 可调节参数（所有参数变化均触发 DLL 同步）
        // ============================================================

        [Header("相机内参")]
        [Tooltip("焦距（像素），影响 Field of View。典型值：600~1200")]
        [Range(200f, 3000f)]
        public float focalLengthPixels = 800f;

        [Tooltip("传感器宽度（mm），物理尺寸")]
        [Range(1f, 36f)]
        public float sensorWidthMM = 6.4f;

        [Tooltip("传感器高度（mm），物理尺寸")]
        [Range(1f, 24f)]
        public float sensorHeightMM = 4.8f;

        [Header("像素当量")]
        [Tooltip("齿轮目标像素当量（mm/pixel）")]
        [Range(0.01f, 1.0f)]
        public float pixelEquivGear = 0.05f;

        [Tooltip("螺母目标像素当量（mm/pixel）")]
        [Range(0.01f, 1.0f)]
        public float pixelEquivNut = 0.04f;

        [Header("算法参数")]
        [Tooltip("小波分解尺度数，影响边缘检测灵敏度")]
        [Range(1, 6)]
        public int waveletScales = 3;

        [Tooltip("Hu矩识别阈值，越小越严格")]
        [Range(0.05f, 1.0f)]
        public float huThreshold = 0.3f;

        [Header("配置文件路径（相对于 Application.streamingAssetsPath）")]
        public string calibrationXmlRelPath = "camera_calibration.xml";
        public string huTemplateCsvRelPath  = "hu_moments_templates.csv";

        [Header("光源参数（影响虚拟场景渲染）")]
        [Tooltip("场景主光源强度")]
        [Range(0f, 8f)]
        public float lightIntensity = 1.0f;

        [Tooltip("场景主光源角度（度）")]
        [Range(0f, 180f)]
        public float lightAngle = 45f;

        // ============================================================
        // 内部缓存（用于检测参数是否发生变化，仅在变化时同步）
        // ============================================================
        private float _prevFocalLength;
        private int   _prevWaveletScales;
        private float _prevHuThreshold;
        private float _prevPixelEquivGear;
        private float _prevPixelEquivNut;

        // 场景主光源引用
        private Light? _mainLight;

        // Unity Camera 组件
        private Camera? _camera;

        // ============================================================
        // Unity 生命周期：初始化
        // ============================================================
        private void Start()
        {
            _camera    = GetComponent<Camera>();
            _mainLight = FindObjectOfType<Light>();

            // 缓存初始值
            _prevFocalLength   = focalLengthPixels;
            _prevWaveletScales = waveletScales;
            _prevHuThreshold   = huThreshold;
            _prevPixelEquivGear = pixelEquivGear;
            _prevPixelEquivNut  = pixelEquivNut;

            // 启动时加载 DLL 配置文件
            LoadDllConfigFiles();

            // 初始同步所有参数到 DLL
            SyncAllParamsToDll();

            Debug.Log($"[CameraController] 初始化完成，DLL 版本：{VisionBridgeUnity.GetVersionString()}");
        }

        // ============================================================
        // Unity 生命周期：每帧更新（检测参数变化并同步）
        // ============================================================
        private void Update()
        {
            // 按需同步：仅在 Inspector 中修改了参数才触发 DLL 调用
            // 避免每帧无意义的 P/Invoke 开销

            if (!Mathf.Approximately(focalLengthPixels, _prevFocalLength))
            {
                SyncFocalLengthToCamera();
                SyncCalibrationToDll();
                _prevFocalLength = focalLengthPixels;
            }

            if (waveletScales != _prevWaveletScales)
            {
                VisionBridgeUnity.SetWaveletScales(waveletScales);
                _prevWaveletScales = waveletScales;
                Debug.Log($"[CameraController] 小波尺度更新：{waveletScales}");
            }

            if (!Mathf.Approximately(huThreshold, _prevHuThreshold))
            {
                VisionBridgeUnity.SetHuThreshold(huThreshold);
                _prevHuThreshold = huThreshold;
                Debug.Log($"[CameraController] Hu矩阈值更新：{huThreshold:F2}");
            }

            if (!Mathf.Approximately(pixelEquivGear, _prevPixelEquivGear) ||
                !Mathf.Approximately(pixelEquivNut,  _prevPixelEquivNut))
            {
                // 像素当量变化时重新生成并加载临时标定 XML
                RegenerateCalibrationXml();
                _prevPixelEquivGear = pixelEquivGear;
                _prevPixelEquivNut  = pixelEquivNut;
            }

            // 同步光源参数到场景
            SyncLightParams();
        }

        // ============================================================
        // 焦距 → Unity Camera Field of View 换算
        //
        // 原理：
        //   FOV_vertical = 2 · arctan(sensorHeight / (2·focalLength))
        //   其中 focalLength 和 sensorHeight 单位须一致
        //   此处 focalLength 为像素单位，需换算为 mm：
        //   focalLengthMM = focalLengthPixels × (sensorHeightMM / imageHeightPixels)
        //   （此处用 sensorHeightMM / 480 作为像素尺寸近似，480 为参考分辨率高度）
        // ============================================================
        private void SyncFocalLengthToCamera()
        {
            if (_camera == null) return;

            // 像素当量（mm/pixel）≈ 传感器高度 / 参考图像高度（像素）
            const float REF_HEIGHT_PX  = 720f;    // 参考分辨率：720p
            float pixelSizeMM = sensorHeightMM / REF_HEIGHT_PX;

            // 焦距（mm）= 焦距（pixel）× 像素尺寸（mm/pixel）
            float focalLengthMM = focalLengthPixels * pixelSizeMM;

            // 垂直 FOV（弧度）= 2 · arctan(h_sensor / (2·f))
            float fovRad = 2f * Mathf.Atan(sensorHeightMM / (2f * focalLengthMM));
            float fovDeg = fovRad * Mathf.Rad2Deg;

            _camera.fieldOfView = fovDeg;

            Debug.Log($"[CameraController] 焦距={focalLengthPixels}px → fMM={focalLengthMM:F2}mm，FOV={fovDeg:F2}°");
        }

        // ============================================================
        // 同步光源参数到 Unity 场景
        // ============================================================
        private void SyncLightParams()
        {
            if (_mainLight == null) return;
            _mainLight.intensity = lightIntensity;
            _mainLight.transform.rotation = Quaternion.Euler(lightAngle, 0, 0);
        }

        // ============================================================
        // 加载 DLL 配置文件（StreamingAssets 目录）
        // ============================================================
        private void LoadDllConfigFiles()
        {
            // StreamingAssets 路径在不同平台有所不同，Unity 自动处理
            string calibPath   = Path.Combine(Application.streamingAssetsPath,
                                              calibrationXmlRelPath);
            string templatePath = Path.Combine(Application.streamingAssetsPath,
                                               huTemplateCsvRelPath);

            if (File.Exists(calibPath))
            {
                VisionBridgeUnity.SetCalibrationParams(calibPath);
                Debug.Log($"[CameraController] 标定文件已加载：{calibPath}");
            }
            else
            {
                Debug.LogWarning($"[CameraController] 标定文件不存在：{calibPath}");
            }

            if (File.Exists(templatePath))
            {
                VisionBridgeUnity.SetHuTemplatePath(templatePath);
                Debug.Log($"[CameraController] Hu矩模板已加载：{templatePath}");
            }
            else
            {
                Debug.LogWarning($"[CameraController] Hu矩模板不存在：{templatePath}");
            }
        }

        // ============================================================
        // 同步所有参数到 DLL（初始化时调用）
        // ============================================================
        private void SyncAllParamsToDll()
        {
            SyncFocalLengthToCamera();
            VisionBridgeUnity.SetWaveletScales(waveletScales);
            VisionBridgeUnity.SetHuThreshold(huThreshold);
        }

        // ============================================================
        // 标定参数变化时通知 DLL
        // ============================================================
        private void SyncCalibrationToDll()
        {
            // 焦距变化时，重新生成标定 XML 并加载
            RegenerateCalibrationXml();
        }

        // ============================================================
        // 动态生成临时标定 XML（当 Inspector 参数手动调整时）
        //
        // 这样 Unity 中调整焦距、传感器尺寸或像素当量后，
        // C++ DLL 端的算法参数也能实时同步更新
        // ============================================================
        private void RegenerateCalibrationXml()
        {
            string tempPath = Path.Combine(Application.temporaryCachePath,
                                           "temp_calibration.xml");

            // 计算相机内参矩阵元素
            // 主点默认在图像中心（cx = W/2, cy = H/2）
            const float REF_WIDTH_PX  = 1280f;
            const float REF_HEIGHT_PX = 720f;
            float cx = REF_WIDTH_PX  / 2f;
            float cy = REF_HEIGHT_PX / 2f;

            // 生成 OpenCV FileStorage 格式的 XML
            string xmlContent = $@"<?xml version=""1.0""?>
<opencv_storage>
  <!-- 动态生成：由 Unity CameraController 参数联动生成 -->
  <!-- 时间戳：{DateTime.Now:yyyy-MM-dd HH:mm:ss} -->
  <camera_matrix type_id=""opencv-matrix"">
    <rows>3</rows>
    <cols>3</cols>
    <dt>d</dt>
    <data>
      {focalLengthPixels:F4} 0. {cx:F4}
      0. {focalLengthPixels:F4} {cy:F4}
      0. 0. 1.
    </data>
  </camera_matrix>
  <dist_coeffs type_id=""opencv-matrix"">
    <rows>1</rows>
    <cols>5</cols>
    <dt>d</dt>
    <data>0. 0. 0. 0. 0.</data>
  </dist_coeffs>
  <pixel_equiv_gear>{pixelEquivGear:F6}</pixel_equiv_gear>
  <pixel_equiv_nut>{pixelEquivNut:F6}</pixel_equiv_nut>
  <pixel_equiv_default>{pixelEquivGear:F6}</pixel_equiv_default>
</opencv_storage>";

            try
            {
                File.WriteAllText(tempPath, xmlContent);
                VisionBridgeUnity.SetCalibrationParams(tempPath);
                Debug.Log($"[CameraController] 标定参数已同步到 DLL（焦距={focalLengthPixels}px，" +
                          $"齿轮当量={pixelEquivGear}mm/px）");
            }
            catch (Exception ex)
            {
                Debug.LogError($"[CameraController] 生成标定 XML 失败：{ex.Message}");
            }
        }

        // ============================================================
        // Unity Editor 辅助：在 Inspector 中修改参数时实时触发
        // （仅在 Editor 模式生效，Build 版本忽略）
        // ============================================================
#if UNITY_EDITOR
        private void OnValidate()
        {
            // 在 Inspector 中拖动滑块时实时更新
            if (Application.isPlaying)
            {
                Update();
            }
        }
#endif

        // ============================================================
        // Gizmos：在 Scene 视图中可视化相机视锥体
        // ============================================================
        private void OnDrawGizmos()
        {
            if (_camera == null) return;
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(transform.position, 0.1f);
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawFrustum(Vector3.zero, _camera.fieldOfView,
                               _camera.farClipPlane, _camera.nearClipPlane,
                               _camera.aspect);
        }
    }
}
