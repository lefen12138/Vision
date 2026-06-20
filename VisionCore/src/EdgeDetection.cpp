/**
 * @file    EdgeDetection.cpp
 * @brief   边缘检测与定位模块 —— 实现
 *
 * 实现思路（依据立项报告第 3.1 节）：
 *
 * 1. detectEdgesByWavelet
 *    - 使用墨西哥帽小波（Mexican Hat Wavelet）频域实现
 *    - 墨西哥帽小波 = 高斯函数二阶导数
 *    - 时域：ψ(x,y) = (2 - r²/σ²) · exp(-r²/(2σ²))
 *    - 频域：Ψ̂(u,v) = (u²+v²)·σ²·exp(-(u²+v²)·σ²/2)
 *    - 多尺度：σ_s = σ_0 · 2^(s-1)，尺度越大捕获越粗的边缘
 *    - 模极大值点（Modulus Maxima）对应图像边缘位置
 *
 * 2. locateGearNut
 *    - Otsu 二值化后用形态学操作清理噪声
 *    - findContours 提取外轮廓
 *    - 面积最大的连通域即目标工件
 *    - minAreaRect 计算最小外接矩形（含旋转角）
 */

#include "EdgeDetection.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>

// ============================================================
// 模块级常量定义（禁止使用魔法数字）
// ============================================================
static constexpr int    DFT_OPT_SIZE_FLAG   = 0;      // getOptimalDFTSize 标志
static constexpr double WAVELET_BASE_SIGMA  = 8.0;    // 基准频率参数（经验值）
static constexpr double MIN_CONTOUR_AREA    = 100.0;  // 最小有效连通域面积（像素²）
static constexpr int    MORPH_KERNEL_SIZE   = 3;      // 形态学操作核大小（像素）

// ============================================================
// 内部辅助函数：构造频域墨西哥帽小波滤波器
//
// 输出为复数型 Mat（CV_64FC2），实部为频域响应，虚部为 0
// 参数 sigma 控制频带中心：sigma 越大 → 低频带 → 粗尺度边缘
// ============================================================
static cv::Mat buildMexicanHatFilter(int rows, int cols, double sigma)
{
    cv::Mat filter(rows, cols, CV_64FC2, cv::Scalar(0.0, 0.0));

    const double centerR = rows / 2.0;
    const double centerC = cols / 2.0;
    const double sigma2  = sigma * sigma;

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            // 归一化频率坐标（以图像中心为原点）
            double u = (c - centerC) / static_cast<double>(cols);
            double v = (r - centerR) / static_cast<double>(rows);

            // 频域半径平方 ρ² = u² + v²
            double rho2 = u * u + v * v;

            // 墨西哥帽频域响应（纯实数带通滤波器）
            // Ψ̂(u,v) = ρ²·σ²·exp(-ρ²·σ²/2)
            double response = rho2 * sigma2 * std::exp(-rho2 * sigma2 / 2.0);

            filter.at<cv::Vec2d>(r, c)[0] = response;  // 实部
            // filter.at<cv::Vec2d>(r, c)[1] = 0.0;    // 虚部（已由 Scalar 初始化为 0）
        }
    }
    return filter;
}

// ============================================================
// 内部辅助函数：频谱中心化（fftshift）
//
// 将 DC 分量从左上角移至图像中心，便于频域滤波操作
// 实现方式：交换四个象限的位置
// ============================================================
static void fftShift(cv::Mat& complexImg)
{
    int cx = complexImg.cols / 2;
    int cy = complexImg.rows / 2;

    // 定义四个象限 ROI（引用同一块内存，直接交换）
    cv::Mat q0(complexImg, cv::Rect(0,  0,  cx, cy));   // 左上
    cv::Mat q1(complexImg, cv::Rect(cx, 0,  cx, cy));   // 右上
    cv::Mat q2(complexImg, cv::Rect(0,  cy, cx, cy));   // 左下
    cv::Mat q3(complexImg, cv::Rect(cx, cy, cx, cy));   // 右下

    // 交换 q0 ↔ q3，q1 ↔ q2
    cv::Mat tmp;
    q0.copyTo(tmp); q3.copyTo(q0); tmp.copyTo(q3);
    q1.copyTo(tmp); q2.copyTo(q1); tmp.copyTo(q2);
}

// ============================================================
// 多尺度小波边缘检测实现
// ============================================================
std::vector<cv::Point> detectEdgesByWavelet(const cv::Mat& src, int scales)
{
    // ---- 步骤 1：灰度化预处理 ----
    cv::Mat gray;
    if (src.channels() == 3)
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    else
        gray = src.clone();

    // 转为 64 位浮点，确保 DFT 精度（避免 8 位量化误差）
    cv::Mat floatImg;
    gray.convertTo(floatImg, CV_64F);

    // ---- 步骤 2：DFT 尺寸扩展（提升 FFT 计算速度）----
    // getOptimalDFTSize 返回 2/3/5 的整数倍，FFT 效率最佳
    int optRows = cv::getOptimalDFTSize(gray.rows);
    int optCols = cv::getOptimalDFTSize(gray.cols);

    cv::Mat padded;
    cv::copyMakeBorder(floatImg, padded,
                       0, optRows - gray.rows,
                       0, optCols - gray.cols,
                       cv::BORDER_CONSTANT, cv::Scalar(0));

    // ---- 步骤 3：构造复数平面并执行 DFT ----
    // OpenCV DFT 要求输入为双通道复数 Mat（实部 + 虚部）
    cv::Mat planes[] = {
        padded,
        cv::Mat::zeros(padded.size(), CV_64F)  // 虚部初始化为 0
    };
    cv::Mat complexImg;
    cv::merge(planes, 2, complexImg);
    cv::dft(complexImg, complexImg, cv::DFT_COMPLEX_OUTPUT);

    // 频谱中心化：使 DC 分量位于图像中心，便于对称滤波
    fftShift(complexImg);

    // ---- 步骤 4：多尺度小波滤波（逐尺度处理并融合）----
    cv::Mat fusedCoeff = cv::Mat::zeros(complexImg.size(), CV_64F);

    for (int s = 1; s <= scales; ++s)
    {
        // 尺度参数：σ_s = σ_0 · 2^(s-1)
        // 尺度增大 → 频带中心下移 → 检测粗尺度边缘结构
        double sigma = WAVELET_BASE_SIGMA * std::pow(2.0, s - 1);

        // 构造当前尺度的频域小波滤波器
        cv::Mat waveletFilter = buildMexicanHatFilter(
            complexImg.rows, complexImg.cols, sigma);

        // 频域乘法（图像频谱 × 小波滤波器）
        // 复数乘以实数：实部虚部各自乘以滤波器实部
        cv::Mat filtered = complexImg.clone();
        for (int r = 0; r < filtered.rows; ++r)
        {
            for (int c = 0; c < filtered.cols; ++c)
            {
                double w = waveletFilter.at<cv::Vec2d>(r, c)[0];
                filtered.at<cv::Vec2d>(r, c)[0] *= w;
                filtered.at<cv::Vec2d>(r, c)[1] *= w;
            }
        }

        // 反中心化（ifftshift），恢复 DC 在左上角的标准布局
        fftShift(filtered);

        // IDFT 恢复空间域小波系数
        cv::Mat iDft;
        cv::dft(filtered, iDft,
                cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_COMPLEX_OUTPUT);

        // 提取实部（小波系数图）
        cv::Mat realPart;
        cv::extractChannel(iDft, realPart, 0);

        // 各尺度绝对值等权叠加（多尺度融合策略）
        cv::Mat absCoeff;
        cv::absdiff(realPart, cv::Scalar(0), absCoeff);   // |coeff|
        fusedCoeff += absCoeff;
    }

    // ---- 步骤 5：裁剪回原始图像尺寸（去除 DFT 填充区域）----
    cv::Mat coeffCropped = fusedCoeff(cv::Rect(0, 0, gray.cols, gray.rows));

    // ---- 步骤 6：归一化并用 Otsu 法自适应提取模极大值点 ----
    cv::Mat coeffNorm;
    cv::normalize(coeffCropped, coeffNorm, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::Mat edgeMask;
    cv::threshold(coeffNorm, edgeMask, 0, 255,
                  cv::THRESH_BINARY | cv::THRESH_OTSU);

    // ---- 步骤 7：形态学细化（闭运算连接断裂边缘）----
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
    cv::morphologyEx(edgeMask, edgeMask, cv::MORPH_CLOSE, kernel);

    // ---- 步骤 8：收集所有边缘点坐标 ----
    std::vector<cv::Point> edgePoints;
    edgePoints.reserve(512);  // 预分配，减少动态扩容开销

    for (int r = 0; r < edgeMask.rows; ++r)
    {
        const uchar* rowPtr = edgeMask.ptr<uchar>(r);
        for (int c = 0; c < edgeMask.cols; ++c)
        {
            if (rowPtr[c] > 0)
                edgePoints.emplace_back(c, r);
        }
    }

    return edgePoints;
}

// ============================================================
// 齿轮/螺母连通域定位实现
// ============================================================
cv::RotatedRect locateGearNut(const cv::Mat& binary)
{
    // ---- 步骤 1：轮廓提取（外部轮廓，CHAIN_APPROX_SIMPLE 压缩冗余点）----
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i>              hierarchy;

    // clone() 避免 findContours 修改原图
    cv::findContours(binary.clone(), contours, hierarchy,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
        std::cerr << "[locateGearNut] 未检测到有效轮廓" << std::endl;
        return cv::RotatedRect();   // 返回空矩形
    }

    // ---- 步骤 2：找面积最大的连通域（代表目标工件）----
    int    maxIdx  = 0;
    double maxArea = 0.0;

    for (int i = 0; i < static_cast<int>(contours.size()); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxIdx  = i;
        }
    }

    // ---- 步骤 3：过滤噪声（面积阈值，单位：像素²）----
    if (maxArea < MIN_CONTOUR_AREA)
    {
        std::cerr << "[locateGearNut] 最大连通域面积 " << maxArea
                  << " < 阈值 " << MIN_CONTOUR_AREA << "，视为噪声" << std::endl;
        return cv::RotatedRect();
    }

    // ---- 步骤 4：计算最小外接矩形 ----
    // minAreaRect 返回 RotatedRect：
    //   .center —— 矩形中心坐标（即目标定位结果）
    //   .size   —— 矩形宽高（不保证 width > height）
    //   .angle  —— 与水平轴夹角，范围 [-90°, 0°]
    cv::RotatedRect minRect = cv::minAreaRect(contours[maxIdx]);

    return minRect;
}
