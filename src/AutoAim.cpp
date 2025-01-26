// 引入头文件
#include "AutoAim.h"

using namespace ly;

int main(int argc, char** argv){
    
    // 初始化串口数据传输参数
    Params_ToSerialPort params_to_serial_port(&SerialParam::recv_data);
    Params_ToVideo params_to_video;
    Params_ToDetector params_to_detector;

    /******* 初始化日志 ********/
    auto log = new Log();  // 创建日志对象
    log->init(argv[0]);    // 初始化日志，传入程序名称
    

    /******* 初始化配置 ********/
    auto config = new Config(confog_file_path); // 创建配置对象并传入配置文件路径
    config->parse();  // 解析配置文件

    // /******* 初始化串口读取 ********/
    // auto serial_port = new SerialPort(SerialParam::device_name);  // 创建串口对象
    // // 创建一个线程用于串口数据读取
    // thread serial_port_thread(&SerialPort::read_data, serial_port, ref(params_to_serial_port));

    /******* 初始化摄像头 ********/
    ly::VideoCapture* video;  // 创建视频捕获对象
    video->chooseCameraType(video);  // 选择摄像头类型
    // 创建一个线程用于视频捕获
    thread video_thread(&ly::VideoCapture::startCapture, video, ref(params_to_video));

    // /******* 初始化目标检测器 ********/
    // auto detector = new Detector();  // 创建目标检测对象
    // detector->setParams(params_to_video, params_to_serial_port);  // 设置目标检测器的参数
    // // 创建一个线程用于启动目标检测
    // thread detector_thread(&Detector::startDetect, detector, ref(params_to_detector), serial_port);

//    /******** 初始化视频保存 ********/
//    if(GlobalParam::SAVE_VIDEO){  // 如果需要保存视频
//         auto saver = new VideoSaver();  // 创建视频保存对象
//         // 创建一个线程用于保存视频
//         thread saver_thread(&VideoSaver::SaveVideo, saver, params_to_video.frame_pp);
//         saver_thread.join();  // 等待视频保存线程完成
//    }

    // 等待所有线程完成
    // serial_port_thread.join();
    video_thread.join();
    std::cout << "video_thread.join();" << std::endl;
    // detector_thread.join();
    return 0;  // 程序结束
}
