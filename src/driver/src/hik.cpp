#include "/home/qianli/buff25/HUST_HeroAim_2024/src/driver/hikSDK/include/MvCameraControl.h"
#include "VideoCapture.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>  // 用于 sleep

using namespace cv;
using namespace ly;

HikCamera::HikCamera() : camera_handle_(nullptr), fail_count_(0), over(false)
{
    std::cout << "启动 HikCamera!" << std::endl;

    // 枚举设备
    MV_CC_DEVICE_INFO_LIST device_list;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (device_list.nDeviceNum == 0)
    {
        std::cerr << "未找到相机!" << std::endl;
        return;
    }

    // 创建相机句柄并打开设备
    nRet = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (nRet != MV_OK)
    {
        std::cerr << "创建句柄失败!" << std::endl;
        return;
    }

    nRet = MV_CC_OpenDevice(camera_handle_);
    if (nRet != MV_OK)
    {
        std::cerr << "打开设备失败!" << std::endl;
        return;
    }

    // 获取相机信息
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    std::cout << "相机信息: 宽度: " << img_info_.nWidthValue
              << ", 高度: " << img_info_.nHeightValue << std::endl;

    // 初始化图像转换参数
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    // convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    convert_param_.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    

    declareParameters();


}

HikCamera::~HikCamera()
{
    if (camera_handle_)
    {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
    }

    std::cout << "HikCamera 已销毁!" << std::endl;
}

void HikCamera::declareParameters()
{
    MVCC_FLOATVALUE f_value;

    // 设置曝光时间
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    double exposure_time = params_.exposure_time; 
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    std::cout << "曝光时间: " << exposure_time << std::endl;

    // 设置增益
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    double gain = params_.gain;  
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    std::cout << "增益: " << gain << std::endl;
}

// void HikCamera::startCapture(Params_ToVideo &params_to_video) 
// {
//     MV_FRAME_OUT out_frame;
//     char input;

//     // 启动图像抓取
//     MV_CC_StartGrabbing(camera_handle_);

//     while (!over)
//     {
//         // 获取图像缓冲区
//         nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
//         if (nRet != MV_OK)
//         {
//             std::cerr << "获取图像缓冲区失败, 错误码: " << std::hex << nRet << std::endl;
//             handleError();
//             continue;
//         }

//         // 设置图像转换参数并转换图像格式
//         convert_param_.pDstBuffer = frame_data_;
//         convert_param_.nDstBufferSize = sizeof(frame_data_);
//         convert_param_.pSrcData = out_frame.pBufAddr;
//         convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
//         convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

//         nRet = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
//         if (nRet != MV_OK)
//         {
//             std::cerr << "像素格式转换失败!" << std::endl;
//             MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
//             continue;
//         }

//         // 使用 OpenCV 显示图像
//         Mat image(out_frame.stFrameInfo.nHeight, out_frame.stFrameInfo.nWidth, CV_8UC3, frame_data_);
//         cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
//         imshow("Hik Camera", image);
//         input = waitKey(1);  // 等待1ms响应键盘输入

//         // 处理退出条件
//         if (input == 'q')
//         {
//             over = true;
//             break;
//         }

//         MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
//         fail_count_ = 0;  // 重置失败计数
//     }
// }

void HikCamera::startCapture(Params_ToVideo &params_to_video) {

    // 启动图像抓取
    MV_CC_StartGrabbing(camera_handle_);
    // 参数传递，将外部传入的帧数据指针引用传递给内部参数
    _video_thread_params.frame_pp = params_to_video.frame_pp;

    // 初始化变量
    int id = 0;  // 用于循环管理帧的索引
    constexpr int size = 10;  // 设置图像帧数组大小为 10

    // 创建帧数据数组，每帧都是一个 Image 对象，包含 cv::Mat 和其他信息
    Image frame[size];  
    for (auto& m : frame) {
        // 为每个 Image 对象中的 mat 分配内存，初始化大小为 1440x1080 的 CV_32FC3 类型的 Mat（3通道，32位浮点数）
        m.mat = new cv::Mat(cv::Size(1440, 1080), CV_32FC3);  // 初始化图像矩阵
    }

    // 声明计时点
    std::chrono::steady_clock::time_point start, end;

    do {
        // 获取图像锁，保证图像数据的线程安全
        std::unique_lock<std::mutex> umtx_video(Thread::mtx_image);

        start = std::chrono::steady_clock::now();  // 捕获当前时间戳，开始计时

        // 捕获图像帧
        MV_FRAME_OUT out_frame;
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);  // 获取图像缓冲区，超时时间为 1000ms
        if (nRet != MV_OK) {  // 如果获取失败，则输出错误并跳过该帧
            std::cerr << "获取图像缓冲区失败! nRet: [" << std::hex << nRet << "]" << std::endl;
            continue;
        }

        // // 将图像数据转换为 RGB 格式
        // // 设置转换参数，将源数据存放到 frame[id].mat 指向的内存空间
        // convert_param_.pDstBuffer = frame[id].mat->data;  // 目标缓冲区指向 frame[id].mat 的数据
        // convert_param_.nDstBufferSize = frame[id].mat->total() * frame[id].mat->elemSize();  // 目标缓冲区大小
        // convert_param_.pSrcData = out_frame.pBufAddr;  // 源数据来自相机的缓冲区
        // convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;  // 源数据长度
        // convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;  // 源数据的像素类型

        // 设置图像转换参数并转换图像格式
        convert_param_.pDstBuffer = frame_data_;
        convert_param_.nDstBufferSize = sizeof(frame_data_);
        convert_param_.pSrcData = out_frame.pBufAddr;
        convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
        convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

        // 执行像素格式转换，将相机图像从原始格式转换为 RGB 格式
        nRet = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
        if (nRet != MV_OK) {  // 如果转换失败，释放图像缓冲区并跳过当前帧
            std::cerr << "像素格式转换失败!" << std::endl;
            MV_CC_FreeImageBuffer(camera_handle_, &out_frame);  // 释放图像缓冲区
            continue;
        }

        // 记录时间戳
        end = std::chrono::steady_clock::now();  // 获取当前时间戳
        double delta_t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;  // 计算帧处理时间（单位：毫秒）
        
        // 计算当前帧的时间戳，存储 IMU 数据（假设 IMU 数据已经接收）
        frame[id].time_stamp = start + (end - start) / 2;  // 设置时间戳为当前时间段的中间点
        frame[id].imu_data = SerialParam::recv_data;  // 将 IMU 数据存储到帧数据中
        frame[id].mat = new cv::Mat(out_frame.stFrameInfo.nHeight, out_frame.stFrameInfo.nWidth, CV_8UC3, frame_data_);


        // Mat image(out_frame.stFrameInfo.nHeight, out_frame.stFrameInfo.nWidth, CV_8UC3, frame_data_);
        // 将当前帧数据存储到 Params_ToVideo 传入的指针中
        *_video_thread_params.frame_pp = &frame[id];

        // 标记图像数据已更新，通知等待的线程
        Thread::image_is_update = true;
        Thread::cond_is_update.notify_one();  // 唤醒等待的线程，表明图像已经准备好

        // 解锁，允许其他线程访问图像
        umtx_video.unlock();

        // 更新帧索引，确保在循环中循环使用有限的帧数组
        id = (id + 1) % size;

        // 检查当前帧的图像是否为空，避免后续操作时出错
        LOG_IF(ERROR, (*_video_thread_params.frame_pp)->mat->empty()) << "获取到空图像矩阵!";

        // // 显示图像，使用 OpenCV 显示当前帧
        // if ((*_video_thread_params.frame_pp)->mat != nullptr && !(*_video_thread_params.frame_pp)->mat->empty()) {
        //     // 使用 OpenCV 显示图像，imshow 会显示在一个窗口中
        //     cv::imshow("Hik Camera", *(*_video_thread_params.frame_pp)->mat);
        //     cv::waitKey(1);  // 等待 1ms，处理显示更新并响应键盘输入
        // }

        // 释放当前帧的图像缓冲区
        MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

    } while (!(*_video_thread_params.frame_pp)->mat->empty());  // 循环条件：直到获取到空图像为止，退出循环
}



// // 错误处理和恢复
// void HikCamera::handleError()
// {
//     std::cout << "获取图像失败，尝试重新启动..." << std::endl;
//     fail_count_++;

//     if (fail_count_ > 5)
//     {
//         std::cerr << "相机故障，重试失败超过 5 次。" << std::endl;
//         over = true;
//     }
//     else
//     {
//         MV_CC_StopGrabbing(camera_handle_);
//         MV_CC_StartGrabbing(camera_handle_);
//     }
// }

void HikCamera::open() {
    // 打开相机（类似于 DaHenCamera 的 open 方法）
    std::cout << "打开 Hik Camera..." << std::endl;
    // 这里可以根据需要添加更多配置（如曝光、增益等）
}