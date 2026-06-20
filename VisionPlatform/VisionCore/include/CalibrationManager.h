/**
 * @file    CalibrationManager.h
 * @brief   相机标定参数管理类 —— 接口声明
 *
 * 功能：
 *   - 从 OpenCV 格式 XML 文件读取内参矩阵、畸变系数、像素当量
 *   - 提供像素距离 ↔ 物理尺寸（毫米）的双向换算接口
 *   - 支持按目标类型（齿轮/螺母）选择对应像素当量
 *
 * 像素当量定义：
 *   像素当量 e（mm/pixel）= 已知物理尺寸（mm）÷ 对应像素数
 *   物理尺寸（mm）= 像素数 × 像素当量 e
 */

#pragma once

#include <string>
#include <opencv2/opencv.hpp>

// ============================================================
// 标定数据结构体
// ============================================================
struct CalibrationData
{
    cv::Mat cameraMatrix;       // 内参矩阵 K（3×3）
    cv::Mat distCoeffs;         // 畸变系数（1×5 或 1×8）

    double pixelEquivGear;      // 齿轮目标像素当量（mm/pixel）
    double pixelEquivNut;       // 螺母目标像素当量（mm/pixel）
    double pixelEquivDefault;   // 默认像素当量（mm/pixel）

    bool   isLoaded;            // 是否已从 XML 成功加载

    // 构造函数：设置工业相机经验初始值（工作距离约 500mm）
    CalibrationData()
        : pixelEquivGear(0.05)
        , pixelEquivNut(0.04)
        , pixelEquivDefault(0.05)
        , isLoaded(false)
    {}
};

// ============================================================
// CalibrationManager —— 标定参数管理类
// ============================================================
class CalibrationManager
{
public:
    CalibrationManager();
    ~CalibrationManager() = default;

    // --------------------------------------------------------
    // 从 XML 文件加载标定参数
    // XML 节点格式（OpenCV FileStorage 标准）：
    //   camera_matrix     —— 内参矩阵
    //   dist_coeffs       —— 畸变系数
    //   pixel_equiv_gear  —— 齿轮像素当量（自定义扩展字段）
    //   pixel_equiv_nut   —— 螺母像素当量（自定义扩展字段）
    //   pixel_equiv_default —— 默认像素当量
    //
    // 返回：加载成功返回 true，文件不存在或格式错误返回 false
    // --------------------------------------------------------
    bool loadFromXML(const std::string& xmlPath);

    // --------------------------------------------------------
    // 像素距离 → 物理尺寸（毫米）
    //
    // 计算公式：physicalMM = pixels × pixelEquiv(target)
    //
    // 参数：
    //   pixels —— 像素距离
    //   target —— 目标类型："gear" / "nut" / 其他（使用 default）
    // --------------------------------------------------------
    double pixelsToMM(double pixels, const std::string& target = "gear");

    // --------------------------------------------------------
    // 物理尺寸（毫米）→ 像素距离（逆向换算）
    // --------------------------------------------------------
    double mmToPixels(double mm, const std::string& target = "gear");

    // --------------------------------------------------------
    // 手动设置像素当量（供 WPF 参数面板手动输入调用）
    // --------------------------------------------------------
    void setPixelEquiv(double equivMM, const std::string& target = "gear");

    // --------------------------------------------------------
    // 获取相机内参矩阵（供 Unity CameraController 读取联动）
    // --------------------------------------------------------
    cv::Mat getCameraMatrix() const;

    // --------------------------------------------------------
    // 获取畸变系数
    // --------------------------------------------------------
    cv::Mat getDistCoeffs() const;

    // --------------------------------------------------------
    // 查询是否已加载有效标定数据
    // --------------------------------------------------------
    bool isCalibrated() const;

private:
    CalibrationData m_calib;   // 内部标定数据
};
