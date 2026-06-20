# 视觉感知虚拟实验平台

> 哈尔滨工业大学（威海）大一年度项目  
> 紫丁香书院·智能制造卓越优才计划

---

## 项目概述

本项目实现一个面向教学的视觉感知虚拟实验平台，支持齿轮、螺母等标准工业零件的：
- **边缘检测与定位**（多尺度小波变换 + 连通域分析）
- **尺寸测量**（像素当量换算 + 公法线长度计算）
- **目标跟踪与识别**（LK 光流 + Hu 矩模板匹配）
- **虚拟仿真联动**（Unity 虚拟场景 ↔ C++ 算法参数双向同步）

---

## 技术架构

```
┌─────────────────────────────────────────────────────────────┐
│               C# WPF 交互界面层（.NET 6+）                    │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────┐   │
│  │ 视频预览      │  │ 检测结果可视  │  │  参数控制面板    │   │
│  │ (OpenCvSharp) │  │ (标注/边框)  │  │  (滑块/输入框)  │   │
│  └──────────────┘  └──────────────┘  └─────────────────┘   │
└───────────────────────┬─────────────────────────────────────┘
                        │  P/Invoke (DLL 导出接口)
┌───────────────────────▼─────────────────────────────────────┐
│                VisionCore.dll（C++17 核心算法层）              │
│  ┌─────────────────┐  ┌───────────────┐  ┌──────────────┐  │
│  │ EdgeDetection   │  │ Measurement   │  │  Tracking    │  │
│  │ 多尺度小波边缘   │  │ 公法线计算    │  │  LK 光流     │  │
│  │ 连通域定位      │  │ 像素当量换算   │  │  Hu矩识别    │  │
│  └─────────────────┘  └───────────────┘  └──────────────┘  │
│           依赖：OpenCV 4.8.x（静态链接）                       │
└───────────────────────┬─────────────────────────────────────┘
                        │  P/Invoke (同一 DLL)
┌───────────────────────▼─────────────────────────────────────┐
│           Unity 2022.3 LTS 虚拟仿真层（C# 脚本）              │
│  CameraController：调节焦距/传感器 → 同步 DLL 标定参数         │
│  虚拟场景：模拟光照、相机位姿、工件位置变化                     │
└─────────────────────────────────────────────────────────────┘
```

---

## 目录结构

```
VisionPlatform/
├── VisionCore/                    # C++ 核心算法库
│   ├── include/
│   │   ├── VisionCore.h           # DLL 导出接口（含 ResultStruct）
│   │   ├── EdgeDetection.h        # 边缘检测与定位
│   │   ├── CalibrationManager.h   # 标定参数管理
│   │   ├── MeasurementModule.h    # 尺寸测量（公法线）
│   │   └── TrackingRecognition.h  # 跟踪与识别
│   ├── src/
│   │   ├── EdgeDetection.cpp      # 多尺度小波边缘检测实现
│   │   ├── CalibrationManager.cpp # XML 标定加载 + 像素当量换算
│   │   ├── MeasurementModule.cpp  # 公法线长度公式实现
│   │   ├── TrackingRecognition.cpp# LK 光流 + Hu 矩识别
│   │   └── VisionCore.cpp         # DLL 导出接口实现
│   └── CMakeLists.txt             # CMake 构建脚本
│
├── VisionWPF/                     # C# WPF 交互界面
│   ├── MainWindow.xaml            # 主界面 XAML（三列布局）
│   ├── MainWindow.xaml.cs         # 主界面代码后台
│   ├── VisionBridge.cs            # P/Invoke 桥接层
│   └── VisionWPF.csproj           # 项目文件（.NET 6 WPF）
│
├── VisionUnity/                   # Unity 虚拟仿真层
│   └── Assets/VisionPlatform/Scripts/
│       └── CameraController.cs    # 相机参数联动脚本
│
├── Config/                        # 配置文件
│   ├── camera_calibration.xml     # 示例相机标定文件
│   └── hu_moments_templates.csv   # Hu 矩模板库（齿轮/螺母）
│
└── README.md                      # 本文件
```

---

## 编译环境要求

### C++ 核心库（VisionCore.dll）
| 依赖 | 版本 | 说明 |
|------|------|------|
| C++ 标准 | C++17 | 必须，MSVC 2019+ 或 MinGW-w64 |
| CMake | 3.16+ | 构建工具 |
| OpenCV | 4.8.x | 静态链接版本 |
| Visual Studio | 2019/2022 | Windows 推荐 |

### C# WPF 界面
| 依赖 | 版本 | 说明 |
|------|------|------|
| .NET | 6.0+ | WPF 框架 |
| OpenCvSharp4.Windows | 4.8.x | NuGet 包，用于视频采集 |

### Unity 仿真层
| 依赖 | 版本 | 说明 |
|------|------|------|
| Unity | 2022.3 LTS | 必须 |
| C# 脚本 | UnityEngine 原生 | 无第三方依赖 |

---

## 编译步骤

### 第一步：编译 VisionCore.dll

```bash
# 1. 克隆/下载项目
cd VisionPlatform/VisionCore

# 2. 创建构建目录
mkdir build && cd build

# 3. 配置 CMake（指定 OpenCV 路径）
cmake -G "Visual Studio 17 2022" -A x64 \
      -DOpenCV_DIR="C:/opencv/build" \
      ..

# 4. 编译（Release 模式）
cmake --build . --config Release

# 输出：build/bin/Release/VisionCore.dll
```

> **提示**：如果没有 OpenCV，可从 [opencv.org](https://opencv.org/releases/) 下载 Windows 预编译包

### 第二步：编译 WPF 应用

```bash
cd VisionPlatform/VisionWPF

# 安装 NuGet 依赖
dotnet restore

# 构建
dotnet build -c Release

# 运行
dotnet run
```

> **重要**：将 `VisionCore.dll` 复制到 WPF 应用程序输出目录（通常为 `bin/Release/net6.0-windows/`）

### 第三步：Unity 项目配置

1. 打开 Unity Hub，创建新的 3D 项目（Unity 2022.3 LTS）
2. 将 `VisionUnity/Assets/` 目录复制到 Unity 项目的 `Assets/` 目录
3. 将编译好的 `VisionCore.dll` 放到 `Assets/Plugins/x86_64/` 目录
4. 将配置文件放到 `Assets/StreamingAssets/` 目录
5. 在场景中创建 Camera 对象，挂载 `CameraController` 脚本
6. 在 Inspector 中调整参数，运行即可

---

## DLL 接口说明

### 核心数据结构

```cpp
#pragma pack(1)
struct ResultStruct {
    double centerX;          // 目标中心 X 坐标（像素）
    double centerY;          // 目标中心 Y 坐标（像素）
    double widthMM;          // 目标宽度（毫米）
    double heightMM;         // 目标高度（毫米）
    double normalLengthMM;   // 齿轮公法线长度（毫米）
    double rotationAngle;    // 旋转角度（度）
    char   className[32];    // 识别类别："gear" / "nut" / "unknown"
};
#pragma pack()
```

### 导出函数

```cpp
// 主处理接口（每帧调用）
void ProcessFrame(unsigned char* data, int width, int height, int step,
                  ResultStruct* outResult);

// 加载标定文件
void SetCalibrationParams(const char* xmlPath);

// 设置小波尺度（1~6）
void SetWaveletScales(int scales);

// 设置 Hu 矩模板路径
void SetHuTemplatePath(const char* csvPath);

// 设置识别阈值（0.05~1.0）
void SetHuThreshold(double threshold);
```

---

## 核心算法说明

### 1. 多尺度小波边缘检测

基于墨西哥帽小波（Mexican Hat Wavelet）的频域实现：

$$\hat{\Psi}(u,v) = \rho^2 \sigma^2 \cdot e^{-\rho^2 \sigma^2 / 2}$$

- 利用 OpenCV DFT 接口实现，无需第三方小波库
- 多尺度融合：等权叠加各尺度绝对值
- Otsu 法自适应提取模极大值边缘点

### 2. 齿轮公法线长度计算

严格实现立项报告公式 (2)：

$$W = m \cdot \cos\alpha \cdot [\pi(k - 0.5) + z \cdot \text{inv}(\alpha)]$$

其中渐开线函数 $\text{inv}(\alpha) = \tan\alpha - \alpha$

- m：模数（mm）
- α：压力角（标准值 20°）
- k：跨测齿数（可自动推荐：$k = \lfloor z\alpha/\pi + 0.5 \rfloor$）
- z：总齿数

### 3. Lucas-Kanade 金字塔光流跟踪

- 调用 `cv::calcOpticalFlowPyrLK`
- 金字塔层数：3 层（约处理 ±30 像素位移）
- 搜索窗口：21×21 像素
- 失效检测：跟踪误差 > 50% 对角线长度时触发重初始化

### 4. Hu 矩模板匹配识别

- 7 阶 Hu 不变矩（平移/旋转/尺度不变）
- 对数空间归一化：$h'_i = \text{sign}(h_i) \cdot \log_{10}(|h_i|)$
- 欧氏距离最近邻分类
- 阈值过滤：距离 > threshold 输出 "unknown"

---

## 配置文件使用

### camera_calibration.xml

```xml
<opencv_storage>
  <camera_matrix type_id="opencv-matrix">
    <!-- [fx  0   cx] -->
    <!-- [0   fy  cy] -->
    <!-- [0   0    1] -->
    <data>800.0 0.0 640.0  0.0 800.0 360.0  0.0 0.0 1.0</data>
  </camera_matrix>
  <pixel_equiv_gear>0.050</pixel_equiv_gear>  <!-- mm/pixel -->
  <pixel_equiv_nut>0.040</pixel_equiv_nut>
</opencv_storage>
```

### hu_moments_templates.csv

```
# 类别名称,hu1,hu2,hu3,hu4,hu5,hu6,hu7
gear,0.1578,0.00012,-0.0000234,...
nut,0.2156,0.00356,-0.000312,...
```

---

## 中期检查交付物

根据立项报告进度安排（2026.03-2026.06 中期），本代码包提供：

| 交付物 | 状态 | 说明 |
|--------|------|------|
| VisionCore.dll 源码 | ✅ 完成 | 全部核心算法，可编译 |
| WPF 界面解决方案 | ✅ 完成 | 主界面 + 参数面板 + P/Invoke |
| Unity 参数联动脚本 | ✅ 完成 | CameraController.cs |
| 示例标定 XML | ✅ 完成 | Config/camera_calibration.xml |
| Hu 矩模板 CSV | ✅ 完成 | Config/hu_moments_templates.csv |

---

## 团队分工

| 成员 | 主要负责 |
|------|---------|
| 魏育杰（组长） | Unity 虚拟仿真平台 |
| 沈恺泽 | WPF 交互界面 |
| 原野 | 图像处理、尺寸测量 |
| 沈伟清 | 目标检测与定位 |
| 林泽楷 | 软件集成、系统测试 |

---

## 参考文献

1. 李祥瑞. 机器视觉研究进展及工业应用综述. 数字通信世界，2021
2. 陶镛泽等. 基于OpenCV和OpenGL的计算机视觉虚拟实验开发. 实验室研究与探索，2021
3. 袁鹏哲等. 基于机器视觉的小模数齿轮精度参数自动化检测技术研究. 制造业自动化，2022
4. OpenCV 4.8 官方文档：https://docs.opencv.org/4.8.0/

---

*指导教师：康文静副教授 | 哈尔滨工业大学（威海）信息科学与工程学院*
