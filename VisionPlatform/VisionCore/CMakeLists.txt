# CMakeLists.txt —— VisionCore.dll 构建脚本
# 目标：编译 C++17 动态链接库，静态链接 OpenCV 4.8.x
# 平台：Windows (MSVC / MinGW)

cmake_minimum_required(VERSION 3.16)
project(VisionCore VERSION 1.0.0 LANGUAGES CXX)

# ============================================================
# C++17 标准（强制，禁止回退到低版本）
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)        # 关闭 GNU 扩展，保持标准兼容

# ============================================================
# OpenCV 4.8.x 静态链接
#
# 使用方法（命令行指定路径）：
#   cmake -G "Visual Studio 17 2022" -A x64 \
#         -DOpenCV_DIR="C:/opencv/build" \
#         ..
#
# 或编辑 OpenCV_DIR 变量直接写入此文件
# ============================================================
# set(OpenCV_DIR "C:/opencv/build")    # ← 取消注释并修改为你的路径

find_package(OpenCV 4.8 REQUIRED COMPONENTS
    core
    imgproc
    imgcodecs
    videoio
    features2d
    calib3d
    video       # calcOpticalFlowPyrLK 所在模块
)

message(STATUS "OpenCV 版本：${OpenCV_VERSION}")
message(STATUS "OpenCV 库：${OpenCV_LIBS}")

# ============================================================
# 源文件列表
# ============================================================
set(VISIONCORE_SOURCES
    src/EdgeDetection.cpp
    src/CalibrationManager.cpp
    src/MeasurementModule.cpp
    src/TrackingRecognition.cpp
    src/VisionCore.cpp
)

set(VISIONCORE_HEADERS
    include/VisionCore.h
    include/EdgeDetection.h
    include/CalibrationManager.h
    include/MeasurementModule.h
    include/TrackingRecognition.h
)

# ============================================================
# 动态库目标
# ============================================================
add_library(VisionCore SHARED
    ${VISIONCORE_SOURCES}
    ${VISIONCORE_HEADERS}
)

# DLL 导出宏（编译 DLL 本体时定义 VISIONCORE_EXPORTS）
target_compile_definitions(VisionCore PRIVATE VISIONCORE_EXPORTS)

# 头文件搜索路径
target_include_directories(VisionCore
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

# 链接 OpenCV 库
target_link_libraries(VisionCore PRIVATE ${OpenCV_LIBS})

# ============================================================
# Windows / MSVC 特定设置
# ============================================================
if(WIN32)
    # 输出目录统一到 build/bin（方便 WPF/Unity 引用 DLL）
    set_target_properties(VisionCore PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        OUTPUT_NAME "VisionCore"
    )

    if(MSVC)
        target_compile_options(VisionCore PRIVATE
            /W4              # 警告级别 4（最严格的常用级别）
            /WX-             # 不将警告升级为错误（教学版宽松设置）
            /utf-8           # 源文件 UTF-8 编码（支持中文注释）
            /Zc:__cplusplus  # 正确报告 __cplusplus 版本（避免误判 C++98）
            /MP              # 多核并行编译（加速大工程构建）
        )

        # Release 模式：启用速度优化
        target_compile_options(VisionCore PRIVATE
            $<$<CONFIG:Release>:/O2 /Ob2>
        )
    endif()
endif()

# ============================================================
# 安装规则（可选，用于打包部署）
# ============================================================
install(TARGETS VisionCore
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(FILES ${VISIONCORE_HEADERS} DESTINATION include/VisionCore)
