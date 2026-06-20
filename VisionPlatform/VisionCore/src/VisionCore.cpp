/**
 * @file    VisionCore.cpp
 * @brief   VisionCore.dll 导出接口实现
 *
 * 本文件实现所有 extern "C" 导出函数，是 C++ 算法层对外的唯一入口。
 * C# WPF 层和 Unity 仿真层均通过 P/Invoke 调用这些接口。
 *
 * 架构图：
 *   [USB 摄像头 / 视频文件]
 *          ↓ 字节流
 *   [ProcessFrame (DLL 入口)]
 *          ↓
 *   ┌───────────────────────────────────┐
 *   │  detectEdgesByWavelet (多尺度小波) │
 *   │  locateGearNut (连通域定位)        │
 *   │  identifyByHuMoments (Hu 矩识别)  │
 *   │  trackByOpticalFlow (LK 光流)      │
 *   │  calculateGearNormalLength (公法线)│
 *   └───────────────────────────────────┘
 *          ↓ ResultStruct
 *   [C# WPF 界面 / Unity 仿真场景]
 */

#include "VisionCore.h"
#include "EdgeDetection.h"
#include "MeasurementModule.h"
#include "TrackingRecognition.h"
#include "CalibrationManager.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <cstring>
#include <algorithm>
#include <iostream>

// ============================================================
// 模块全局状态（DLL 生命周期内持久化）
// ============================================================
static CalibrationManager g_calibMgr;           // 标定参数管理器（单例）
static int                g_waveletScales = 3;  // 当前小波尺度数（默认 3）
static std::string        g_huCsvPath;           // Hu 矩模板 CSV 路径
static double             g_huThreshold   = 0.3; // Hu 矩识别阈值（默认 0.3）

static cv::Mat            g_prevGrayFrame;       // 光流跟踪：上一帧灰度图
static cv::Point2f        g_trackPoint;          // 光流跟踪：当前目标坐标

// DLL 版本字符串（静态，调用方无需释放）
static const char* const DLL_VERSION = "VisionCore 1.0.0 | 哈工大(威海) 大一年度项目";

// 形态学操作内核大小（必须为奇数）
static constexpr int MORPH_KERNEL_SIZE = 5;

// 高斯模糊核大小（预处理降噪，必须为奇数）
static constexpr int BLUR_KERNEL_SIZE = 5;

// 最小有效目标面积（像素²），用于过滤极小噪声块
static constexpr double MIN_VALID_AREA_PX2 = 200.0;

// ============================================================
// 导出接口：加载相机标定 XML 文件
// ============================================================
extern "C" __declspec(dllexport)
void SetCalibrationParams(const char* xmlPath)
{
    if (xmlPath == nullptr)
    {
        std::cerr << "[SetCalibrationParams] xmlPath 为空指针" << std::endl;
        return;
    }
    g_calibMgr.loadFromXML(std::string(xmlPath));
}

// ============================================================
// 导出接口：设置小波分解尺度数
// ============================================================
extern "C" __declspec(dllexport)
void SetWaveletScales(int scales)
{
    // 限制范围 [1, 6]，防止参数越界导致性能崩溃
    const int MIN_SCALES = 1;
    const int MAX_SCALES = 6;
    g_waveletScales = std::max(MIN_SCALES, std::min(scales, MAX_SCALES));
    std::cout << "[SetWaveletScales] 小波尺度更新为：" << g_waveletScales << std::endl;
}

// ============================================================
// 导出接口：设置 Hu 矩模板文件路径
// ============================================================
extern "C" __declspec(dllexport)
void SetHuTemplatePath(const char* csvPath)
{
    if (csvPath != nullptr)
    {
        g_huCsvPath = std::string(csvPath);
        std::cout << "[SetHuTemplatePath] 模板路径：" << g_huCsvPath << std::endl;
    }
}

// ============================================================
// 导出接口：设置 Hu 矩识别阈值
// ============================================================
extern "C" __declspec(dllexport)
void SetHuThreshold(double threshold)
{
    g_huThreshold = threshold;
    std::cout << "[SetHuThreshold] 识别阈值：" << g_huThreshold << std::endl;
}

// ============================================================
// 导出接口：获取 DLL 版本字符串
// ============================================================
extern "C" __declspec(dllexport)
const char* GetVersion()
{
    return DLL_VERSION;
}

// ============================================================
// 导出接口：主图像处理入口
//
// 执行顺序：
//   1. 包装字节流为 OpenCV Mat（零拷贝）
//   2. 灰度化 + 高斯模糊 + Otsu 二值化
//   3. 形态学操作去噪
//   4. 多尺度小波边缘检测
//   5. 连通域定位（最大连通域最小外接矩形）
//   6. 目标识别（Hu 矩最近邻匹配）
//   7. 尺寸换算（像素当量）
//   8. 光流跟踪更新
//   9. 齿轮公法线计算（当识别为齿轮时）
//  10. 填充输出结构体
// ============================================================
extern "C" __declspec(dllexport)
void ProcessFrame(
    unsigned char* data,
    int            width,
    int            height,
    int            step,
    ResultStruct*  outResult)
{
    // ---- 参数检查 ----
    if (data == nullptr || outResult == nullptr || width <= 0 || height <= 0)
    {
        std::cerr << "[ProcessFrame] 无效参数" << std::endl;
        return;
    }

    // ---- 初始化输出结构体 ----
    std::memset(outResult, 0, sizeof(ResultStruct));
    std::strncpy(outResult->className, "unknown",
                 sizeof(outResult->className) - 1);

    try
    {
        // ----  步骤 1：零拷贝包装图像（step 处理行对齐 padding）----
        cv::Mat frame(height, width, CV_8UC3, data,
                      static_cast<size_t>(step));

        // ---- 步骤 2：灰度化预处理 ----
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 高斯模糊抑制传感器噪声
        cv::GaussianBlur(gray, gray,
                         cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);

        // ---- 步骤 3：Otsu 自适应二值化（背景亮时用 BINARY_INV）----
        cv::Mat binary;
        cv::threshold(gray, binary, 0, 255,
                      cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

        // 形态学开运算：去除小噪声连通域
        cv::Mat morphKernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, morphKernel);

        // 形态学闭运算：填充目标内部空洞
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, morphKernel);

        // ---- 步骤 4：多尺度小波边缘检测 ----
        // 边缘点集可用于可视化（当前处理流程中暂不直接使用）
        // std::vector<cv::Point> edgePts = detectEdgesByWavelet(frame, g_waveletScales);

        // ---- 步骤 5：连通域定位 ----
        cv::RotatedRect boundRect = locateGearNut(binary);

        // 有效性检查：面积过小视为无目标
        double roiArea = boundRect.size.width * boundRect.size.height;
        if (roiArea < MIN_VALID_AREA_PX2)
        {
            std::cout << "[ProcessFrame] 未检测到有效目标" << std::endl;
            return;
        }

        // 填充位置信息
        outResult->centerX      = boundRect.center.x;
        outResult->centerY      = boundRect.center.y;
        outResult->rotationAngle = boundRect.angle;

        // ---- 步骤 6：目标识别（Hu 矩）----
        // 从定位矩形截取 ROI
        cv::Rect roiRect = boundRect.boundingRect();
        roiRect &= cv::Rect(0, 0, width, height);   // 边界裁剪，防止越界

        std::string className = "unknown";
        if (roiRect.width > 8 && roiRect.height > 8)
        {
            cv::Mat roi = frame(roiRect);
            className   = identifyByHuMoments(roi, g_huCsvPath, g_huThreshold);
        }

        std::strncpy(outResult->className, className.c_str(),
                     sizeof(outResult->className) - 1);

        // ---- 步骤 7：像素尺寸 → 物理尺寸换算 ----
        outResult->widthMM  = g_calibMgr.pixelsToMM(boundRect.size.width,  className);
        outResult->heightMM = g_calibMgr.pixelsToMM(boundRect.size.height, className);

        // ---- 步骤 8：光流跟踪 ----
        cv::Mat currGray;
        cv::cvtColor(frame, currGray, cv::COLOR_BGR2GRAY);

        if (!g_prevGrayFrame.empty() &&
            g_trackPoint.x > 0 && g_trackPoint.y > 0)
        {
            g_trackPoint = trackByOpticalFlow(
                g_prevGrayFrame, currGray, g_trackPoint);
        }
        else
        {
            // 首帧：以定位中心初始化跟踪点
            g_trackPoint = boundRect.center;
        }
        currGray.copyTo(g_prevGrayFrame);

        // ---- 步骤 9：齿轮公法线计算 ----
        // 注：齿轮参数（z, m, k）应由 WPF 界面传入，
        //     此处使用估算值作为演示（实际项目中从参数管理器获取）
        if (className == "gear")
        {
            // 根据外接矩形宽度估算模数（工程近似：齿顶圆直径 ≈ m(z+2)）
            const int    EST_TOOTH_COUNT = 20;   // 演示用齿数（应由 WPF 传入）
            double       estModule       = outResult->widthMM / (EST_TOOTH_COUNT + 2.0);
            int          spanTeeth       = recommendSpanTeeth(EST_TOOTH_COUNT);

            outResult->normalLengthMM = calculateGearNormalLength(
                EST_TOOTH_COUNT, estModule, spanTeeth);
        }
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "[ProcessFrame] OpenCV 异常：" << e.what() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ProcessFrame] 标准异常：" << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "[ProcessFrame] 未知异常" << std::endl;
    }
}
