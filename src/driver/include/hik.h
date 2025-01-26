#ifndef HIK_CAMERA_H
#define HIK_CAMERA_H

#include "/home/qianli/buff25/HUST_HeroAim_2024/src/driver/src/ros2-hik-camera-main/hikSDK/include/MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

namespace hik
{
    class HikCamera : public VideoCapture
    {
    public:
        // 构造函数，初始化相机
        HikCamera();

        // 析构函数，销毁相机资源
        ~HikCamera();

        // 打开相机
        void open();

        // 开始图像捕获
        void startCapture();

    private:
        // 相机句柄
        void* camera_handle_;

        // 返回值
        int nRet;

        // 相机图像基本信息
        MV_IMAGE_BASIC_INFO img_info_;

        // 图像转换参数
        MV_CC_PIXEL_CONVERT_PARAM convert_param_;

        // 用于存储图像数据的缓冲区
        char frame_data_[1920 * 1080 * 3];  // 假设 1080p 分辨率

        // 连续失败次数，用于处理异常
        int fail_count_;

        // 捕获图像的线程
        std::thread capture_thread_;
    };
}  // namespace hik

#endif  // HIK_CAMERA_H
