/**
 * @file    TrackingRecognition.h
 * @brief   目标跟踪与识别模块 —— 接口声明
 *
 * 本模块实现立项报告第 3.3 节的两类功能：
 *
 * 【跟踪】Lucas-Kanade 金字塔光流算法
 *   - 假设小邻域内像素运动一致（brightness constancy constraint）
 *   - 通过最小化前后帧亮度误差求解光流方程组
 *   - 金字塔策略：由粗到细，处理大位移目标
 *   - 调用 OpenCV calcOpticalFlowPyrLK 接口
 *
 * 【识别】Hu 矩模板匹配
 *   - Hu 矩具有平移、旋转、尺度不变性，适合工件形状分类
 *   - 加载预存 CSV 模板库（含齿轮/螺母两类）
 *   - 计算当前 ROI 与各模板的欧氏距离（对数空间归一化）
 *   - 按阈值输出最近邻识别结果
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// ============================================================
// Hu 矩模板数据结构
// ============================================================
struct HuMomentsTemplate
{
    std::string className;    // 类别名称（"gear" / "nut"）
    double      huMoments[7]; // 7 阶 Hu 不变矩特征值
};

// ============================================================
// Lucas-Kanade 金字塔光流目标跟踪
//
// 调用说明：
//   - 前后帧均可为灰度或 BGR，内部自动转灰度
//   - 首帧调用时用定位中心初始化 initPt
//   - 后续帧将上一帧返回坐标作为 initPt 持续调用
//
// 参数：
//   prev   —— 前一帧图像（CV_8UC1 或 CV_8UC3）
//   curr   —— 当前帧图像（CV_8UC1 或 CV_8UC3）
//   initPt —— 目标在前一帧中的位置坐标
//
// 返回值：
//   目标在当前帧中的预测坐标；跟踪失败时返回 initPt（保持不动）
// ============================================================
cv::Point2f trackByOpticalFlow(
    const cv::Mat&    prev,
    const cv::Mat&    curr,
    const cv::Point2f& initPt
);

// ============================================================
// 基于 Hu 矩的零件模板匹配识别
//
// 流程：
//   1. ROI 灰度化 + Otsu 二值化
//   2. 计算图像矩 moments() 和 Hu 不变矩 HuMoments()
//   3. 对数空间欧氏距离与所有模板逐一比对
//   4. 取最小距离对应类别，距离超阈值则返回 "unknown"
//
// 参数：
//   roi       —— 目标感兴趣区域（BGR 或灰度图）
//   csvPath   —— Hu 矩模板 CSV 文件路径（空则用已缓存模板）
//   threshold —— 判定阈值，默认 0.3
//
// 返回值：
//   识别类别字符串："gear" / "nut" / "unknown"
// ============================================================
std::string identifyByHuMoments(
    const cv::Mat&    roi,
    const std::string& csvPath   = "",
    double             threshold = 0.3
);

// ============================================================
// 从 CSV 文件加载 Hu 矩模板库
//
// CSV 格式（每行）：
//   类别名称,hu1,hu2,hu3,hu4,hu5,hu6,hu7
//   以 '#' 开头的行视为注释，跳过
//
// 参数：
//   csvPath —— CSV 文件路径
//
// 返回值：
//   模板列表（可能为空，若文件不存在或格式错误）
// ============================================================
std::vector<HuMomentsTemplate> loadHuTemplatesFromCSV(
    const std::string& csvPath
);

// ============================================================
// 计算两组 Hu 矩的对数空间欧氏距离
//
// 对数变换公式（解决各分量数值范围差异问题）：
//   h'_i = sign(hu_i) × log₁₀(|hu_i|)   若 hu_i ≠ 0
//   h'_i = 0                              若 hu_i = 0
//
// 距离公式：
//   d = √(Σ(h'_i(1) - h'_i(2))²)   i ∈ [0,6]
//
// 参数：
//   hu1[7] —— 第一组 Hu 矩
//   hu2[7] —— 第二组 Hu 矩
// ============================================================
double computeHuDistance(const double hu1[7], const double hu2[7]);
