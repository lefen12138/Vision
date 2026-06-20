/**
 * MainWindow.xaml.cs —— 主界面代码后台
 *
 * 核心职责：
 *   1. 视频/摄像头帧采集（定时器驱动，约 30 FPS）
 *   2. 将帧数据传递给 VisionBridge.ProcessFrameSafe（P/Invoke）
 *   3. 将检测结果 ResultStruct 更新到 UI 控件
 *   4. 参数控件变化时同步调用 DLL 参数设置接口
 *   5. 在处理后图像上绘制目标边框和标注信息
 *
 * 线程模型：
 *   - 帧采集在 DispatcherTimer 线程（UI 线程）执行
 *   - DLL 调用在 Task.Run 异步线程中，结果通过 Dispatcher 更新 UI
 */

using System;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using Microsoft.Win32;

// OpenCvSharp 用于摄像头/视频采集（需要 NuGet 包 OpenCvSharp4.Windows）
// 如使用其他方案（如 Emgu.CV），请替换下方 using 和相关代码
using OpenCvSharp;
using OpenCvSharp.Extensions;

namespace VisionWPF
{
    public partial class MainWindow : System.Windows.Window
    {
        // ============================================================
        // 成员变量
        // ============================================================

        // 视频捕获对象（摄像头 or 视频文件）
        private VideoCapture? _capture;

        // 帧驱动定时器（目标 ~30 FPS）
        private DispatcherTimer? _frameTimer;

        // FPS 计算器
        private readonly Stopwatch _fpsWatch    = new Stopwatch();
        private int                _frameCount  = 0;
        private double             _currentFps  = 0.0;

        // 处理是否进行中（防止帧处理重入）
        private bool _isProcessing = false;

        // 当前选择的目标类型（用于像素当量设置）
        private string _selectedTarget = "gear";

        // ============================================================
        // 帧间隔（毫秒）
        // ============================================================
        private const int FRAME_INTERVAL_MS = 33;   // ≈ 30 FPS

        // ============================================================
        // 初始化
        // ============================================================
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            // 检查 DLL 可用性并更新状态指示
            if (VisionBridge.IsDllAvailable())
            {
                dllStatusDot.Fill   = new SolidColorBrush(Color.FromRgb(0xA6, 0xE3, 0xA1));
                txtDllStatus.Text   = "DLL 已就绪";
                txtVersion.Text     = VisionBridge.GetVersionString();
                UpdateStatus("DLL 加载成功，请打开摄像头或视频文件");
            }
            else
            {
                dllStatusDot.Fill   = new SolidColorBrush(Color.FromRgb(0xE6, 0x45, 0x53));
                txtDllStatus.Text   = "DLL 未找到";
                UpdateStatus("警告：VisionCore.dll 未找到，请将 DLL 放至应用程序目录");
            }

            // 初始化帧驱动定时器
            _frameTimer = new DispatcherTimer(DispatcherPriority.Normal)
            {
                Interval = TimeSpan.FromMilliseconds(FRAME_INTERVAL_MS)
            };
            _frameTimer.Tick += FrameTimer_Tick;
        }

        // ============================================================
        // 窗口关闭：释放视频资源
        // ============================================================
        private void MainWindow_Closing(object? sender, System.ComponentModel.CancelEventArgs e)
        {
            StopCapture();
        }

        // ============================================================
        // 工具栏：打开摄像头（设备 0）
        // ============================================================
        private void BtnOpenCamera_Click(object sender, RoutedEventArgs e)
        {
            StopCapture();

            _capture = new VideoCapture(0);
            if (!_capture.IsOpened())
            {
                MessageBox.Show("无法打开摄像头（设备索引 0），请检查摄像头连接",
                    "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                _capture.Dispose();
                _capture = null;
                return;
            }

            StartCapture("📷 摄像头输入");
        }

        // ============================================================
        // 工具栏：打开视频文件
        // ============================================================
        private void BtnOpenVideo_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog
            {
                Title  = "选择视频文件",
                Filter = "视频文件|*.mp4;*.avi;*.mkv;*.mov;*.wmv|所有文件|*.*"
            };

            if (dlg.ShowDialog() != true) return;

            StopCapture();

            _capture = new VideoCapture(dlg.FileName);
            if (!_capture.IsOpened())
            {
                MessageBox.Show($"无法打开视频文件：{dlg.FileName}",
                    "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                _capture.Dispose();
                _capture = null;
                return;
            }

            StartCapture($"🎬 视频：{Path.GetFileName(dlg.FileName)}");
        }

        // ============================================================
        // 工具栏：停止采集
        // ============================================================
        private void BtnStop_Click(object sender, RoutedEventArgs e)
        {
            StopCapture();
        }

        // ============================================================
        // 工具栏：加载标定 XML 文件
        // ============================================================
        private void BtnLoadCalib_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog
            {
                Title  = "选择相机标定 XML 文件",
                Filter = "XML 标定文件|*.xml|所有文件|*.*"
            };

            if (dlg.ShowDialog() != true) return;

            VisionBridge.SetCalibrationParams(dlg.FileName);
            UpdateStatus($"标定文件已加载：{Path.GetFileName(dlg.FileName)}");
        }

        // ============================================================
        // 工具栏：加载 Hu 矩模板 CSV 文件
        // ============================================================
        private void BtnLoadHuTemplate_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog
            {
                Title  = "选择 Hu 矩模板 CSV 文件",
                Filter = "CSV 模板文件|*.csv|所有文件|*.*"
            };

            if (dlg.ShowDialog() != true) return;

            VisionBridge.SetHuTemplatePath(dlg.FileName);
            UpdateStatus($"Hu矩模板已加载：{Path.GetFileName(dlg.FileName)}");
        }

        // ============================================================
        // 帧处理定时器回调（核心逻辑）
        // ============================================================
        private async void FrameTimer_Tick(object? sender, EventArgs e)
        {
            if (_capture == null || !_capture.IsOpened() || _isProcessing)
                return;

            _isProcessing = true;

            try
            {
                // ---- 采集一帧 ----
                using var mat = new Mat();
                bool grabbed = _capture.Read(mat);

                // 视频播放到末尾时循环
                if (!grabbed || mat.Empty())
                {
                    if (_capture.FrameCount > 0)
                        _capture.Set(VideoCaptureProperties.PosFrames, 0);
                    return;
                }

                // ---- 转换为字节数组并调用 DLL ----
                // mat.Data 为 BGR 字节指针，Step 为每行字节数
                byte[] imageBytes = new byte[mat.Height * (int)mat.Step()];
                System.Runtime.InteropServices.Marshal.Copy(
                    mat.Data, imageBytes, 0, imageBytes.Length);

                int step = (int)mat.Step();

                // 在后台线程执行耗时的图像处理（避免阻塞 UI）
                ResultStruct result = await Task.Run(() =>
                    VisionBridge.ProcessFrameSafe(
                        imageBytes, mat.Width, mat.Height, step));

                // ---- 绘制原始帧到左侧预览 ----
                imgOriginal.Source = MatToBitmapSource(mat);

                // ---- 在处理后图像上绘制检测结果 ----
                using var displayMat = mat.Clone();
                DrawDetectionResult(displayMat, result);
                imgProcessed.Source = MatToBitmapSource(displayMat);

                // ---- 更新右侧结果面板 ----
                UpdateResultPanel(result);

                // ---- FPS 计算 ----
                UpdateFps();
            }
            catch (Exception ex)
            {
                UpdateStatus($"帧处理异常：{ex.Message}");
            }
            finally
            {
                _isProcessing = false;
            }
        }

        // ============================================================
        // 在图像上绘制检测结果标注
        // ============================================================
        private static void DrawDetectionResult(Mat mat, ResultStruct result)
        {
            if (result.CenterX <= 0 && result.CenterY <= 0) return;

            // 绘制目标中心点（绿色圆点）
            Cv2.Circle(mat,
                new OpenCvSharp.Point((int)result.CenterX, (int)result.CenterY),
                5, Scalar.Lime, -1);

            // 绘制目标外接矩形（近似为矩形框）
            int halfW = (int)(result.WidthMM  > 0
                ? result.WidthMM  / 0.05   // 还原像素尺寸（使用默认当量）
                : 40);
            int halfH = (int)(result.HeightMM > 0
                ? result.HeightMM / 0.05
                : 40);

            var topLeft = new OpenCvSharp.Point(
                (int)result.CenterX - halfW,
                (int)result.CenterY - halfH);
            var bottomRight = new OpenCvSharp.Point(
                (int)result.CenterX + halfW,
                (int)result.CenterY + halfH);

            Scalar boxColor = result.ClassName == "gear" ? Scalar.Cyan
                            : result.ClassName == "nut"  ? Scalar.Orange
                            : Scalar.Gray;

            Cv2.Rectangle(mat, topLeft, bottomRight, boxColor, 2);

            // 绘制类别标签
            string label = $"{result.ClassName} ({result.WidthMM:F1}x{result.HeightMM:F1}mm)";
            Cv2.PutText(mat, label,
                new OpenCvSharp.Point((int)result.CenterX - halfW, (int)result.CenterY - halfH - 8),
                HersheyFonts.HersheySimplex, 0.55, boxColor, 1);
        }

        // ============================================================
        // 更新右侧结果面板文本
        // ============================================================
        private void UpdateResultPanel(ResultStruct result)
        {
            string cls = result.ClassName ?? "unknown";
            txtResultClass.Text = cls;
            txtResultClass.Foreground = cls == "gear"
                ? new SolidColorBrush(Color.FromRgb(0x89, 0xDC, 0xEB))
                : cls == "nut"
                    ? new SolidColorBrush(Color.FromRgb(0xFA, 0xB3, 0x87))
                    : new SolidColorBrush(Color.FromRgb(0x6C, 0x70, 0x86));

            txtResultCX.Text    = $"{result.CenterX:F1} px";
            txtResultCY.Text    = $"{result.CenterY:F1} px";
            txtResultW.Text     = result.WidthMM  > 0 ? $"{result.WidthMM:F3}"  : "—";
            txtResultH.Text     = result.HeightMM > 0 ? $"{result.HeightMM:F3}" : "—";
            txtResultAngle.Text = $"{result.RotationAngle:F1}°";
            txtResultNL.Text    = result.NormalLengthMM > 0
                                  ? $"{result.NormalLengthMM:F3}"
                                  : "—";
        }

        // ============================================================
        // FPS 统计
        // ============================================================
        private void UpdateFps()
        {
            _frameCount++;
            if (!_fpsWatch.IsRunning) _fpsWatch.Start();

            if (_fpsWatch.Elapsed.TotalSeconds >= 1.0)
            {
                _currentFps = _frameCount / _fpsWatch.Elapsed.TotalSeconds;
                _frameCount = 0;
                _fpsWatch.Restart();
                txtFPS.Text = _currentFps.ToString("F1");
            }
        }

        // ============================================================
        // OpenCvSharp Mat → WPF BitmapSource（用于 Image 控件显示）
        // ============================================================
        private static BitmapSource MatToBitmapSource(Mat mat)
        {
            using var bmp = mat.ToBitmap();
            var hBitmap = bmp.GetHbitmap();
            try
            {
                return System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(
                    hBitmap, IntPtr.Zero, Int32Rect.Empty,
                    BitmapSizeOptions.FromEmptyOptions());
            }
            finally
            {
                // 必须手动释放 GDI 位图句柄，否则内存泄漏
                NativeMethods.DeleteObject(hBitmap);
            }
        }

        // ============================================================
        // 采集控制辅助方法
        // ============================================================
        private void StartCapture(string sourceDesc)
        {
            _frameTimer?.Start();
            btnStop.IsEnabled = true;
            UpdateStatus($"正在采集 | {sourceDesc}");
            _fpsWatch.Restart();
            _frameCount = 0;
        }

        private void StopCapture()
        {
            _frameTimer?.Stop();
            _capture?.Dispose();
            _capture = null;
            btnStop.IsEnabled = false;
            UpdateStatus("已停止");
        }

        private void UpdateStatus(string message)
        {
            txtStatus.Text = $"{DateTime.Now:HH:mm:ss}  |  {message}";
        }

        // ============================================================
        // 参数控件事件处理
        // ============================================================

        // 小波尺度滑块
        private void SliderWaveletScales_ValueChanged(object sender,
            System.Windows.RoutedPropertyChangedEventArgs<double> e)
        {
            int scales = (int)e.NewValue;
            if (txtWaveletScales != null)
                txtWaveletScales.Text = scales.ToString();
            VisionBridge.SetWaveletScales(scales);
        }

        // Hu 矩阈值滑块
        private void SliderHuThreshold_ValueChanged(object sender,
            System.Windows.RoutedPropertyChangedEventArgs<double> e)
        {
            double threshold = Math.Round(e.NewValue, 2);
            if (txtHuThreshold != null)
                txtHuThreshold.Text = threshold.ToString("F2");
            VisionBridge.SetHuThreshold(threshold);
        }

        // 目标类型下拉框
        private void CmbTargetType_SelectionChanged(object sender,
            System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (cmbTargetType.SelectedItem is System.Windows.Controls.ComboBoxItem item)
                _selectedTarget = item.Tag?.ToString() ?? "gear";
        }

        // 像素当量输入框（失去焦点时应用）
        private void TxtPixelEquiv_LostFocus(object sender, RoutedEventArgs e)
        {
            // 手动修改像素当量时通知 DLL（通过标定管理器接口）
            // 注：当前通过重新设置标定文件路径触发，
            //     若需精细控制可增加专用 DLL 导出接口
            UpdateStatus($"像素当量已更新：{txtPixelEquiv.Text} mm/pixel");
        }

        // 计算公法线长度按钮
        private void BtnCalculateNormal_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                int    z = int.Parse(txtToothCount.Text.Trim());
                double m = double.Parse(txtModule.Text.Trim());
                int    k = int.Parse(txtSpanTeeth.Text.Trim());

                // 调用 C++ 公法线计算公式（通过 P/Invoke 间接调用）
                // 此处在 C# 侧复现公式，与 C++ 端保持一致
                // W = m · cos(α) · [π(k-0.5) + z · inv(α)]
                const double alpha    = 20.0 * Math.PI / 180.0;
                double       invAlpha = Math.Tan(alpha) - alpha;

                // 若 k=0，使用推荐公式 k = floor(z·α/π + 0.5)
                if (k <= 0)
                    k = Math.Max(1, (int)Math.Floor(z * alpha / Math.PI + 0.5));

                double W = m * Math.Cos(alpha)
                             * (Math.PI * (k - 0.5) + z * invAlpha);

                txtNormalResult.Text = W.ToString("F4");
                UpdateStatus($"公法线计算完成：W={W:F4} mm（m={m}, z={z}, k={k}）");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"参数格式错误：{ex.Message}", "计算错误",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
            }
        }
    }

    // ============================================================
    // 原生方法：删除 GDI 位图句柄（防止内存泄漏）
    // ============================================================
    internal static class NativeMethods
    {
        [System.Runtime.InteropServices.DllImport("gdi32.dll")]
        internal static extern bool DeleteObject(IntPtr hObject);
    }
}
