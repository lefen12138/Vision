/**
 * @file    CalibrationManager.cpp
 * @brief   相机标定参数管理类 —— 实现
 *
 * XML 文件格式示例（OpenCV FileStorage 标准格式）：
 *   <?xml version="1.0"?>
 *   <opencv_storage>
 *     <camera_matrix type_id="opencv-matrix">...</camera_matrix>
 *     <dist_coeffs type_id="opencv-matrix">...</dist_coeffs>
 *     <pixel_equiv_gear>0.05</pixel_equiv_gear>
 *     <pixel_equiv_nut>0.04</pixel_equiv_nut>
 *     <pixel_equiv_default>0.05</pixel_equiv_default>
 *   </opencv_storage>
 */

#include "CalibrationManager.h"
#include <iostream>
#include <stdexcept>

// ============================================================
// 构造函数：使用工业相机经验值初始化
// （工作距离约 500mm，典型工业 USB 摄像头参数）
// ============================================================
CalibrationManager::CalibrationManager()
{
    // 默认内参矩阵（1280×720 分辨率，焦距约 800 像素，典型值）
    m_calib.cameraMatrix = (cv::Mat_<double>(3, 3)
        << 800.0,   0.0, 640.0,
              0.0, 800.0, 360.0,
              0.0,   0.0,   1.0);

    // 默认畸变系数（假设镜头无显著畸变）
    m_calib.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    // 像素当量经验初始值
    m_calib.pixelEquivGear    = 0.05;    // 齿轮：0.05 mm/pixel
    m_calib.pixelEquivNut     = 0.04;    // 螺母：0.04 mm/pixel
    m_calib.pixelEquivDefault = 0.05;    // 默认：0.05 mm/pixel

    m_calib.isLoaded = false;
}

// ============================================================
// 从 XML 文件加载标定参数
// ============================================================
bool CalibrationManager::loadFromXML(const std::string& xmlPath)
{
    try
    {
        cv::FileStorage fs(xmlPath, cv::FileStorage::READ);

        if (!fs.isOpened())
        {
            std::cerr << "[CalibrationManager] 无法打开标定文件：" << xmlPath << std::endl;
            return false;
        }

        // ---- 读取相机内参矩阵 ----
        cv::Mat camMatrix;
        fs["camera_matrix"] >> camMatrix;
        if (!camMatrix.empty())
            m_calib.cameraMatrix = camMatrix;

        // ---- 读取畸变系数 ----
        cv::Mat distCoeffs;
        fs["dist_coeffs"] >> distCoeffs;
        if (!distCoeffs.empty())
            m_calib.distCoeffs = distCoeffs;

        // ---- 读取像素当量（自定义扩展字段）----
        double pixEquivGear    = 0.0;
        double pixEquivNut     = 0.0;
        double pixEquivDefault = 0.0;

        fs["pixel_equiv_gear"]    >> pixEquivGear;
        fs["pixel_equiv_nut"]     >> pixEquivNut;
        fs["pixel_equiv_default"] >> pixEquivDefault;

        // 仅在 XML 字段存在且有效（> 0）时才覆盖默认值
        if (pixEquivGear    > 0.0) m_calib.pixelEquivGear    = pixEquivGear;
        if (pixEquivNut     > 0.0) m_calib.pixelEquivNut     = pixEquivNut;
        if (pixEquivDefault > 0.0) m_calib.pixelEquivDefault = pixEquivDefault;

        fs.release();
        m_calib.isLoaded = true;

        // 输出加载摘要，方便调试确认
        std::cout << "[CalibrationManager] 标定文件加载成功：" << xmlPath << std::endl;
        std::cout << "  齿轮像素当量：" << m_calib.pixelEquivGear    << " mm/pixel" << std::endl;
        std::cout << "  螺母像素当量：" << m_calib.pixelEquivNut     << " mm/pixel" << std::endl;
        std::cout << "  默认像素当量：" << m_calib.pixelEquivDefault << " mm/pixel" << std::endl;

        return true;
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "[CalibrationManager] OpenCV 异常：" << e.what() << std::endl;
        return false;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[CalibrationManager] 标准异常：" << e.what() << std::endl;
        return false;
    }
}

// ============================================================
// 像素距离 → 物理尺寸（毫米）
// 计算公式：physicalMM = pixels × e（像素当量，mm/pixel）
// ============================================================
double CalibrationManager::pixelsToMM(double pixels, const std::string& target)
{
    double equivMM = m_calib.pixelEquivDefault;

    if      (target == "gear") equivMM = m_calib.pixelEquivGear;
    else if (target == "nut")  equivMM = m_calib.pixelEquivNut;

    return pixels * equivMM;
}

// ============================================================
// 物理尺寸（毫米）→ 像素距离（逆向换算）
// 计算公式：pixels = physicalMM ÷ e
// ============================================================
double CalibrationManager::mmToPixels(double mm, const std::string& target)
{
    double equivMM = m_calib.pixelEquivDefault;

    if      (target == "gear") equivMM = m_calib.pixelEquivGear;
    else if (target == "nut")  equivMM = m_calib.pixelEquivNut;

    return (equivMM > 0.0) ? (mm / equivMM) : 0.0;
}

// ============================================================
// 手动设置像素当量（WPF 参数面板手动输入时调用）
// ============================================================
void CalibrationManager::setPixelEquiv(double equivMM, const std::string& target)
{
    if      (target == "gear") m_calib.pixelEquivGear    = equivMM;
    else if (target == "nut")  m_calib.pixelEquivNut     = equivMM;
    else                       m_calib.pixelEquivDefault = equivMM;
}

cv::Mat CalibrationManager::getCameraMatrix() const { return m_calib.cameraMatrix; }
cv::Mat CalibrationManager::getDistCoeffs()   const { return m_calib.distCoeffs;   }
bool    CalibrationManager::isCalibrated()    const { return m_calib.isLoaded;     }
