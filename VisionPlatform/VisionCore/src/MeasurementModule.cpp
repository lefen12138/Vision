/**
 * @file    MeasurementModule.cpp
 * @brief   尺寸测量模块 —— 实现
 *
 * 公法线长度公式（来源：立项报告公式 (2)）：
 *
 *   W = m · cos(α) · [π(k - 0.5) + z · inv(α)]
 *
 * 其中各符号的工程含义：
 *   W     —— 公法线长度（mm），即跨 k 个齿时与异侧齿廓相切
 *             的两平行平面间的距离，是齿轮精度检测核心指标
 *   m     —— 模数（mm），标准化系列：1, 1.25, 1.5, 2, 2.5, 3...
 *   α     —— 压力角，标准直齿圆柱齿轮取 20°
 *   k     —— 跨测齿数，影响测量精度与可操作性
 *   z     —— 总齿数
 *   inv(α) —— 渐开线函数 = tan(α) - α（α 取弧度）
 *             渐开线函数描述了从基圆切线展开到切触点时
 *             展开角与压力角之差，是渐开线齿轮几何的基础
 */

#include "MeasurementModule.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// ============================================================
// 数学常量（禁止使用未命名魔法数字）
// ============================================================
static constexpr double PI                   = 3.14159265358979323846;
static constexpr double DEG_TO_RAD           = PI / 180.0;
static constexpr double STD_PRESSURE_ANGLE   = 20.0;   // 标准压力角（度）
static constexpr int    MIN_SPAN_TEETH       = 1;       // 跨测齿数下限
static constexpr double MIN_TOOTH_COUNT      = 1.0;     // 最小有效齿数
static constexpr double MIN_MODULE           = 1e-6;    // 最小有效模数（防除零）

// ============================================================
// 渐开线函数实现：inv(α) = tan(α) - α
//
// 推导：
//   渐开线上任意点的向径 r = r_b / cos(φ)，φ 为该点压力角
//   从基圆起，展开角 θ = tan(φ) - φ，即 inv(φ)
//   在 α = 20° 时：inv(20°) = tan(20°) - π/9 ≈ 0.01490
// ============================================================
double involuteFunction(double pressureAngleDeg)
{
    double alphaRad = pressureAngleDeg * DEG_TO_RAD;
    return std::tan(alphaRad) - alphaRad;
}

// ============================================================
// 推荐跨测齿数计算
//
// 公式来源：《机械工程手册》标准测量规范
//   k_recommend = ⌊z · α / π + 0.5⌋
//
// 直觉理解：
//   跨测齿数 ≈ 使公法线测量点落在齿面上（不在齿根/齿顶）的最优选择
//   k 太小：测量不稳定；k 太大：卡规无法放入
// ============================================================
int recommendSpanTeeth(int toothCount, double pressureAngleDeg)
{
    double alphaRad = pressureAngleDeg * DEG_TO_RAD;

    // 按公式取整（四舍五入）
    int k = static_cast<int>(std::floor(
        static_cast<double>(toothCount) * alphaRad / PI + 0.5));

    // 确保跨测齿数在合理范围内
    k = std::max(MIN_SPAN_TEETH, std::min(k, toothCount / 2));

    return k;
}

// ============================================================
// 齿轮公法线长度计算核心实现
//
// 公式（立项报告公式 (2)）：
//   W = m · cos(α) · [π(k - 0.5) + z · inv(α)]
//
// 数值计算验证示例（m=2, z=20, k=3, α=20°）：
//   cos(20°) ≈ 0.9397
//   inv(20°) = tan(20°) - π/9 ≈ 0.3640 - 0.3491 ≈ 0.01490
//   W = 2 × 0.9397 × [π(3-0.5) + 20×0.01490]
//     = 1.8794 × [7.854 + 0.298]
//     = 1.8794 × 8.152
//     ≈ 15.32 mm
// ============================================================
double calculateGearNormalLength(int toothCount, double moduleMM, int spanTeeth)
{
    // ---- 参数有效性验证 ----
    if (toothCount < static_cast<int>(MIN_TOOTH_COUNT) || moduleMM < MIN_MODULE)
    {
        std::cerr << "[calculateGearNormalLength] 无效参数："
                  << "齿数=" << toothCount << ", 模数=" << moduleMM << std::endl;
        return 0.0;
    }

    // ---- 若 spanTeeth <= 0，使用自动推荐值 ----
    int k = (spanTeeth > 0)
            ? spanTeeth
            : recommendSpanTeeth(toothCount, STD_PRESSURE_ANGLE);

    // ---- 角度换算 ----
    double alphaRad = STD_PRESSURE_ANGLE * DEG_TO_RAD;   // 压力角（弧度）

    // ---- 计算渐开线函数值 ----
    double invAlpha = involuteFunction(STD_PRESSURE_ANGLE);

    // ---- 代入公法线公式 ----
    // W = m · cos(α) · [π(k - 0.5) + z · inv(α)]
    double W = moduleMM
             * std::cos(alphaRad)
             * (PI * (k - 0.5) + static_cast<double>(toothCount) * invAlpha);

    std::cout << "[calculateGearNormalLength] "
              << "m=" << moduleMM << ", z=" << toothCount
              << ", k=" << k << ", α=" << STD_PRESSURE_ANGLE << "°"
              << " → W=" << W << " mm" << std::endl;

    return W;
}
