/**
 * VisionBridge.cs —— C# 到 VisionCore.dll 的 P/Invoke 桥接层
 *
 * 通过 Platform Invoke (P/Invoke) 技术，从托管 C# 代码
 * 调用非托管 C++ DLL 中导出的函数。
 *
 * 关键注意事项：
 *   1. ResultStruct 必须加 [StructLayout(Sequential, Pack=1)]
 *      以匹配 C++ 端 #pragma pack(1) 的内存布局
 *   2. 字符串参数用 CharSet.Ansi（C++ 端为 const char*）
 *   3. GCHandle.Alloc + Pinned 防止 GC 在 P/Invoke 期间移动字节数组
 */

using System;
using System.Runtime.InteropServices;

namespace VisionWPF
{
    // ============================================================
    // 统一结果结构体（C# 托管版本）
    //
    // 内存布局必须与 C++ ResultStruct 完全一致：
    //   [StructLayout(Sequential, Pack=1)] 对应 #pragma pack(1)
    //   [MarshalAs(ByValTStr, SizeConst=32)] 对应 char className[32]
    // ============================================================
    [StructLayout(LayoutKind.Sequential, Pack = 1, CharSet = CharSet.Ansi)]
    public struct ResultStruct
    {
        public double CenterX;          // 目标中心 X 坐标（像素）
        public double CenterY;          // 目标中心 Y 坐标（像素）
        public double WidthMM;          // 目标宽度（毫米）
        public double HeightMM;         // 目标高度（毫米）
        public double NormalLengthMM;   // 齿轮公法线长度（毫米）
        public double RotationAngle;    // 旋转角度（度）

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)]
        public string ClassName;        // 识别类别
    }

    // ============================================================
    // VisionCore DLL P/Invoke 声明
    // ============================================================
    public static class VisionBridge
    {
        // DLL 文件名（运行时从应用程序目录查找 VisionCore.dll）
        private const string DLL_NAME = "VisionCore.dll";

        /// <summary>
        /// 主图像处理接口：输入 BGR 帧数据，输出检测结果
        /// </summary>
        /// <param name="data">图像字节指针（BGR，8位）</param>
        /// <param name="width">图像宽度（像素）</param>
        /// <param name="height">图像高度（像素）</param>
        /// <param name="step">每行字节数（含对齐 padding）</param>
        /// <param name="outResult">输出结果结构体（调用方分配）</param>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void ProcessFrame(
            IntPtr        data,
            int           width,
            int           height,
            int           step,
            out ResultStruct outResult
        );

        /// <summary>
        /// 加载相机标定 XML 文件（UTF-8 路径字符串）
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl,
                   CharSet = CharSet.Ansi)]
        public static extern void SetCalibrationParams(string xmlPath);

        /// <summary>
        /// 设置小波分解尺度数（由参数面板滑块联动调用）
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetWaveletScales(int scales);

        /// <summary>
        /// 设置 Hu 矩模板 CSV 文件路径
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl,
                   CharSet = CharSet.Ansi)]
        public static extern void SetHuTemplatePath(string csvPath);

        /// <summary>
        /// 设置 Hu 矩识别阈值
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetHuThreshold(double threshold);

        /// <summary>
        /// 获取 DLL 版本字符串
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr GetVersion();

        /// <summary>
        /// 获取版本字符串（托管包装）
        /// </summary>
        public static string GetVersionString()
        {
            IntPtr ptr = GetVersion();
            return Marshal.PtrToStringAnsi(ptr) ?? "Unknown";
        }

        // ============================================================
        // 安全封装方法：从 byte[] 调用 ProcessFrame
        //
        // 使用 GCHandle.Pinned 固定托管字节数组，
        // 防止 GC 在 P/Invoke 调用期间移动内存
        // ============================================================
        public static ResultStruct ProcessFrameSafe(
            byte[] imageData,
            int    width,
            int    height,
            int    step)
        {
            var result = new ResultStruct();

            if (imageData == null || imageData.Length == 0)
                return result;

            GCHandle handle = GCHandle.Alloc(imageData, GCHandleType.Pinned);
            try
            {
                IntPtr ptr = handle.AddrOfPinnedObject();
                ProcessFrame(ptr, width, height, step, out result);
            }
            finally
            {
                // 无论是否异常，必须释放 GCHandle（防止内存泄漏）
                handle.Free();
            }

            return result;
        }

        // ============================================================
        // 辅助方法：检查 VisionCore.dll 是否可用
        // ============================================================
        public static bool IsDllAvailable()
        {
            try
            {
                string ver = GetVersionString();
                return !string.IsNullOrEmpty(ver);
            }
            catch (DllNotFoundException)
            {
                return false;
            }
            catch (EntryPointNotFoundException)
            {
                return false;
            }
        }
    }
}
