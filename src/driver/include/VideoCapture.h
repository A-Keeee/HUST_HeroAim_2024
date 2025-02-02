//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_VIDEOCAPTURE_H
#define AUTOAIM_VIDEOCAPTURE_H

#include "/home/qianli/buff25/HUST_HeroAim_2024/src/driver/hikSDK/include/MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include "Params.h"
#include "Log.h"
#include "SerialPort.h"
#include "GxCamera.h"
#include "Thread.h"

//#define TIMEIT

using namespace cv;

namespace ly
{

    void *saveFrameToNative(void *params_p);
    void *getFrameFromPicture(void *params_p);

    // 相机类型
    enum CameraType
    {
        DaHen,              // 大恒
        Video,              // 视频 debug
        Picture,            // 图片 debug
        hik,                // 海康相机
    };

    // Image 包含了图像、时间戳以及imu数据       追求时钟同步
    class Image
    {
    public:
        Mat* mat;                                           // 图像
        std::chrono::steady_clock::time_point time_stamp;   // 时间戳
        SerialPortData imu_data;                            // imu_data
    };
    
    struct Params_ToVideo
    {
        cv::VideoCapture video;        // 相机
        Image **frame_pp;              // Image
        void *__this;
        VideoWriter writer;            // 写入 

        Params_ToVideo()
        {
            frame_pp = (Image **)malloc(sizeof(Image *));
            *frame_pp = new Image();
        }
        ~Params_ToVideo(){
            free(*frame_pp);
            free(frame_pp);
        }
    };

    /**
     * @brief:
     */
    class VideoCapture
    {
    public:
        VideoCapture();
        ~VideoCapture();
        virtual void open() = 0;
        virtual void startCapture(Params_ToVideo &) = 0;
        void startSave(Params_ToVideo &params_to_video);
        void chooseCameraType(VideoCapture *&);

    protected:
        double rate{};  // 帧率
        int _id;
        uint16_t height;
        uint16_t width;
        uint16_t offset_x;
        uint16_t offset_y;

        pthread_t threadID{};   // 捕获线程
        pthread_t threadID2{};  // 写入线程

        Params_ToVideo _video_thread_params;

        VideoWriter writer;

        SerialPort *_serial_port;
        SerialPortData *_data_read;
    };

    class DaHenCamera : public VideoCapture
    {
    public:
        explicit DaHenCamera();
        ~DaHenCamera();
        void startCapture(Params_ToVideo &) override;
        void open() override;

    private:
        GxCamera *camera;
        //        int _id;
    };

    class NativeVideo : public VideoCapture
    {
    public:
        explicit NativeVideo();
        ~NativeVideo();
        void open() override;
        void startCapture(Params_ToVideo &params) override;

    private:
        cv::VideoCapture video;
    };

    class NativePicture : public VideoCapture
    {
    public:
        explicit NativePicture() = default;
        void open() override;
        void startCapture(Params_ToVideo &) override;

    private:
        string base_dir;
        string suffix;
        int id = 0;
    };


    // 摄像头参数结构体
    struct HikCameraParams
    {
        double exposure_time = 5000;  // 默认曝光时间
        double gain = 16;             // 默认增益
    };
    // 新增的 HikCamera 类
    class HikCamera : public VideoCapture
    {
    public:
        HikCamera();
        ~HikCamera();
        // void startCapture();
        void startCapture(Params_ToVideo &) override;
        void open() override;
        void declareParameters(); 
        void handleError();
        HikCameraParams params_;  // 相机的参数

    private:
        bool over = false;

        // 相机句柄
        void* camera_handle_;

        // 返回值
        int nRet;

        // 相机图像基本信息
        MV_IMAGE_BASIC_INFO img_info_;

        // 图像转换参数
        MV_CC_PIXEL_CONVERT_PARAM convert_param_;

        // 用于存储图像数据的缓冲区
        unsigned char frame_data_[1440 * 1080 * 3];  // 假设 1080p 分辨率

        // 连续失败次数，用于处理异常
        int fail_count_;

    };

}

#endif //AUTOAIM_VIDEOCAPTURE_H
