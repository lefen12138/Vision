/**
 * @file    TrackingRecognition.cpp
 * @brief   目标跟踪与识别模块 —— 实现
 *
 * 实现两类算法（依据立项报告第 3.3 节）：
 *
 * 【LK 光流跟踪】
 *   - 亮度恒常性假设：I(x,y,t) = I(x+u, y+v, t+1)
 *   - 泰勒展开得光流方程：Ix·u + Iy·v + It = 0
 *   - LK 算法：假设窗口内运动一致，最小二乘求解 [u,v]
 *   - 金字塔：从低分辨率（粗）到高分辨率（细）逐层精化，
 *             有效处理 > 1 像素的大位移目标
 *
 * 【Hu 矩识别】
 *   - Hu 矩（1962）具有平移/旋转/尺度不变性
 *   - 7 阶：φ₁~φ₇，由中心矩和归一化矩推导
 *   - 对数空间：h'ᵢ = sign(φᵢ)·log₁₀(|φᵢ|) 解决数量级差异
 *   - 欧氏距离最近邻分类
 */

#include "TrackingRecognition.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

// ============================================================
// 光流跟踪参数常量
// ============================================================
static constexpr int    LK_PYRAMID_LEVELS  = 3;      // 金字塔层数（3 层可处理约 ±30 像素位移）
static constexpr int    LK_WINDOW_SIZE     = 21;     // 搜索窗口（奇数，推荐 15-31）
static constexpr int    LK_MAX_ITERATIONS  = 30;     // 最大迭代次数（TermCriteria）
static constexpr double LK_EPSILON         = 0.01;   // 收敛精度（像素）
static constexpr float  LK_MAX_ERROR_RATIO = 0.5f;   // 最大误差比例（相对对角线长度）

// ============================================================
// Lucas-Kanade 光流跟踪实现
// ============================================================
cv::Point2f trackByOpticalFlow(
    const cv::Mat&    prev,
    const cv::Mat&    curr,
    const cv::Point2f& initPt)
{
    // ---- 有效性检查 ----
    if (prev.empty() || curr.empty())
        return initPt;

    // ---- 步骤 1：确保输入为灰度图（LK 算法要求单通道）----
    cv::Mat prevGray, currGray;

    auto toGray = [](const cv::Mat& src, cv::Mat& dst) {
        if (src.channels() == 3)
            cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
        else
            dst = src;
    };
    toGray(prev, prevGray);
    toGray(curr, currGray);

    // ---- 步骤 2：配置 LK 参数 ----
    // TermCriteria：满足"达到最大迭代次数"或"误差小于精度阈值"之一即停止
    cv::TermCriteria termCrit(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
        LK_MAX_ITERATIONS, LK_EPSILON);

    cv::Size windowSize(LK_WINDOW_SIZE, LK_WINDOW_SIZE);

    // ---- 步骤 3：调用 OpenCV 金字塔 LK 光流 ----
    std::vector<cv::Point2f> prevPts = { initPt };
    std::vector<cv::Point2f> currPts;
    std::vector<uchar>       status;   // 1 = 跟踪成功，0 = 失败
    std::vector<float>       errors;   // 跟踪残差

    cv::calcOpticalFlowPyrLK(
        prevGray, currGray,
        prevPts,  currPts,
        status,   errors,
        windowSize,
        LK_PYRAMID_LEVELS,
        termCrit,
        0,           // flags（默认）
        1e-4         // minEigThreshold：过滤奇异（退化）特征窗口
    );

    // ---- 步骤 4：验证跟踪结果 ----
    if (status.empty() || status[0] == 0)
    {
        std::cerr << "[trackByOpticalFlow] 光流跟踪失败，目标可能移出视野" << std::endl;
        return initPt;   // 返回原位，触发上层重初始化
    }

    // 误差过大时视为跟踪失效
    // 阈值 = LK_MAX_ERROR_RATIO × 图像对角线长度
    float diagLen = std::sqrt(
        static_cast<float>(prev.cols * prev.cols + prev.rows * prev.rows));

    if (errors[0] > LK_MAX_ERROR_RATIO * diagLen)
    {
        std::cerr << "[trackByOpticalFlow] 跟踪误差过大：" << errors[0]
                  << "（阈值：" << LK_MAX_ERROR_RATIO * diagLen << "）" << std::endl;
        return initPt;
    }

    return currPts[0];
}

// ============================================================
// 从 CSV 文件加载 Hu 矩模板库
// ============================================================
std::vector<HuMomentsTemplate> loadHuTemplatesFromCSV(const std::string& csvPath)
{
    std::vector<HuMomentsTemplate> templates;

    std::ifstream file(csvPath);
    if (!file.is_open())
    {
        std::cerr << "[loadHuTemplatesFromCSV] 无法打开模板文件：" << csvPath << std::endl;
        return templates;
    }

    std::string line;
    int lineNum = 0;

    while (std::getline(file, line))
    {
        ++lineNum;

        // 跳过空行和注释行（以 '#' 开头）
        if (line.empty() || line[0] == '#')
            continue;

        std::stringstream ss(line);
        std::string       token;
        HuMomentsTemplate tmpl{};
        int fieldIdx = 0;
        bool parseError = false;

        while (std::getline(ss, token, ','))
        {
            if (fieldIdx == 0)
            {
                // 第 0 列：类别名称
                tmpl.className = token;
            }
            else if (fieldIdx >= 1 && fieldIdx <= 7)
            {
                // 第 1-7 列：Hu 矩数值
                try
                {
                    tmpl.huMoments[fieldIdx - 1] = std::stod(token);
                }
                catch (...)
                {
                    std::cerr << "[loadHuTemplatesFromCSV] 第 " << lineNum
                              << " 行第 " << fieldIdx << " 列解析失败：" << token << std::endl;
                    parseError = true;
                    break;
                }
            }
            ++fieldIdx;
        }

        // 确保读取到类别名 + 7 个 Hu 矩（fieldIdx >= 8）
        if (!parseError && fieldIdx >= 8)
            templates.push_back(tmpl);
    }

    std::cout << "[loadHuTemplatesFromCSV] 已加载 " << templates.size()
              << " 个模板（来自：" << csvPath << "）" << std::endl;

    return templates;
}

// ============================================================
// Hu 矩对数空间欧氏距离计算
//
// 对数归一化：h'ᵢ = sign(hᵢ) × log₁₀(|hᵢ|)
// 原因：Hu 矩各阶数值差异极大（φ₁~1e0，φ₇~1e-10），
//       直接欧氏距离会导致数值大的分量主导匹配结果
// ============================================================
double computeHuDistance(const double hu1[7], const double hu2[7])
{
    double sumSq = 0.0;

    for (int i = 0; i < 7; ++i)
    {
        // 对数空间归一化（处理零值防止 log(0) 崩溃）
        auto logNorm = [](double h) -> double {
            return (h != 0.0)
                ? std::copysign(std::log10(std::abs(h)), h)
                : 0.0;
        };

        double lh1 = logNorm(hu1[i]);
        double lh2 = logNorm(hu2[i]);
        double diff = lh1 - lh2;
        sumSq += diff * diff;
    }

    return std::sqrt(sumSq);
}

// ============================================================
// 静态模板缓存（避免每帧都重复解析 CSV 文件）
// ============================================================
static std::vector<HuMomentsTemplate> g_huTemplates;
static std::string                    g_lastCsvPath;
static double                         g_threshold = 0.3;

// ============================================================
// 基于 Hu 矩的零件识别实现
// ============================================================
std::string identifyByHuMoments(
    const cv::Mat&    roi,
    const std::string& csvPath,
    double             threshold)
{
    // ---- 步骤 1：按需加载模板（带路径缓存）----
    if (!csvPath.empty() && csvPath != g_lastCsvPath)
    {
        g_huTemplates = loadHuTemplatesFromCSV(csvPath);
        g_lastCsvPath = csvPath;
    }

    g_threshold = threshold;

    if (g_huTemplates.empty())
    {
        std::cerr << "[identifyByHuMoments] 模板库为空，无法识别" << std::endl;
        return "unknown";
    }

    // ---- 步骤 2：ROI 预处理（灰度化 + Otsu 二值化）----
    cv::Mat gray;
    if (roi.channels() == 3)
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    else
        gray = roi;

    // 尺寸过小时直接返回（防止矩计算不稳定）
    if (gray.rows < 8 || gray.cols < 8)
        return "unknown";

    cv::Mat binary;
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // ---- 步骤 3：计算图像矩 ----
    cv::Moments m = cv::moments(binary, true);  // true = 输入为二值图

    // ---- 步骤 4：计算 Hu 不变矩（7 阶）----
    double huMoments[7];
    cv::HuMoments(m, huMoments);

    // ---- 步骤 5：最近邻匹配 ----
    double      minDist   = std::numeric_limits<double>::max();
    std::string bestClass = "unknown";

    for (const auto& tmpl : g_huTemplates)
    {
        double dist = computeHuDistance(huMoments, tmpl.huMoments);
        if (dist < minDist)
        {
            minDist   = dist;
            bestClass = tmpl.className;
        }
    }

    // ---- 步骤 6：阈值判定 ----
    if (minDist > threshold)
    {
        std::cout << "[identifyByHuMoments] 最小距离 " << minDist
                  << " > 阈值 " << threshold << " → 输出 unknown" << std::endl;
        return "unknown";
    }

    std::cout << "[identifyByHuMoments] 识别结果：" << bestClass
              << "（距离=" << minDist << "）" << std::endl;

    return bestClass;
}
