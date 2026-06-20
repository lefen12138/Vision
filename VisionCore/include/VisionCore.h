/**
 * @file    VisionCore.h
 * @brief   视觉感知虚拟实验平台 —— DLL 主导出接口
 * @author  哈尔滨工业大学（威海）大一年度项目组
 *
 * 三层协同标准化接口：
 *   C++ 算法层  <->  C# WPF 层  <->  Unity 仿真层
 *
 * 编译说明：
 *   - 编译 DLL 时定义 VISIONCORE_EXPORTS
 *   - 消费者（C#/Unity）直接包含本头文件即可，宏自动切换
 */

#pragma once

#ifdef VISIONCORE_EXPORTS
    #define VISION_API __declspec(dllexport)
#else
    #define VISION_API __declspec(dllimport)
#endif

#include <cstdint>

// ============================================================
// 统一结果结构体
// #pragma pack(1) 保证跨层调用时内存布局严格一致，
// 防止 C++/C# 编译器各自插入对齐填充字节导致数据错位
// ============================================================
#pragma pack(1)
struct ResultStruct
{
    double centerX;          // 目标中心 X 坐标（像素）
    double centerY;          // 目标中心 Y 坐标（像素）
    double widthMM;          // 目标宽度（毫米，已通过像素当量换算）
    double heightMM;         // 目标高度（毫米，已通过像素当量换算）
    double normalLengthMM;   // 齿轮公法线长度（毫米，仅齿轮目标有效）
    double rotationAngle;    // 目标旋转角度（度，相对水平方向）
    char   className[32];    // 识别类别："gear" / "nut" / "unknown"
};
#pragma pack()


// ============================================================
// DLL 导出接口 —— 主图像处理入口
//
// 功能：输入一帧 BGR 图像，执行边缘检测→连通域定位→尺寸测量
//       →目标识别→光流跟踪全流程，结果通过 outResult 回传
//
// 参数：
//   data      —— 图像原始字节指针（BGR 字节序，8 位无符号）
//   width     —— 图像宽度（像素）
//   height    —— 图像高度（像素）
//   step      —— 每行字节数（= width * 3 + 可能的行对齐 padding）
//   outResult —— 调用方分配的结果结构体指针
// ============================================================
extern "C" VISION_API void ProcessFrame(
    unsigned char* data,
    int            width,
    int            height,
    int            step,
    ResultStruct*  outResult
);

// ============================================================
// DLL 导出接口 —— 加载相机标定参数
//
// 功能：解析 OpenCV 格式的标定 XML 文件，
//       更新内部 CalibrationManager 的内参矩阵与像素当量
//
// 参数：
//   xmlPath —— 标定文件绝对路径（UTF-8 字符串）
// ============================================================
extern "C" VISION_API void SetCalibrationParams(const char* xmlPath);

// ============================================================
// DLL 导出接口 —— 设置小波分解尺度数
//
// 功能：由 WPF 参数面板的滑块调用，动态调整边缘检测灵敏度
//       尺度越大，可检测越粗的边缘结构；默认值 3
//
// 参数：
//   scales —— 小波分解层数，有效范围 [1, 6]
// ============================================================
extern "C" VISION_API void SetWaveletScales(int scales);

// ============================================================
// DLL 导出接口 —— 设置 Hu 矩模板文件路径
//
// 功能：指定存放齿轮/螺母 Hu 矩特征的 CSV 文件路径
//
// 参数：
//   csvPath —— CSV 文件绝对路径（UTF-8 字符串）
// ============================================================
extern "C" VISION_API void SetHuTemplatePath(const char* csvPath);

// ============================================================
// DLL 导出接口 —— 设置 Hu 矩识别阈值
//
// 功能：当欧氏距离超过此阈值时，识别结果为 "unknown"
//
// 参数：
//   threshold —— 阈值，推荐范围 [0.1, 1.0]，默认 0.3
// ============================================================
extern "C" VISION_API void SetHuThreshold(double threshold);

// ============================================================
// DLL 导出接口 —— 获取版本字符串
//
// 返回值：指向静态字符串的指针，调用方无需释放
// ============================================================
extern "C" VISION_API const char* GetVersion();
