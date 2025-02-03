// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_AUTOAIM_H
#define AUTOAIM_AUTOAIM_H

// 引入必需的头文件，包括配置、日志、视频捕获、视频保存、目标检测等模块
#include "utils/include/Config.h"
#include "utils/include/Log.h"
#include "driver/include/VideoCapture.h"
#include "driver/include/VideoSaver.h"
#include "armor_detector/include/Detector.h"
#include "Thread.h"
#include "trainSVM.h"

#include <string>

using namespace std;
using namespace ly;

namespace ly{
    // 配置文件路径
    const string config_file_path = "../src/utils/tools/init.json";
}

#endif //AUTOAIM_AUTOAIM_H
