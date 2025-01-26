# find_package(OpenCV 4 REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/include/opencv4/")
set(OpenCV_DIR /usr/local/include/opencv4/)
# set(OpenVINO_LIBRARIES "/opt/openvino/runtime/lib/intel64/libopenvino.so")
set(OpenVINO_INCLUDE_DIRS "/opt/openvino/runtime/include")
find_package(OpenCV 4 REQUIRED)
set(InferenceEngine_DIR /opt/openvino/runtime/cmake)
set(NGRAPH_DIR /opt/openvino/runtime/cmake)
find_package(OpenVINO 2024 REQUIRED)
# find_package(ngraph REQUIRED)
find_package(Eigen3 REQUIRED)



#find_package(PythonLibs 3.8 REQUIRED)
# set(PYTHON_LIBRARY "/usr/lib/python3.5/config-3.5m-aarch64-linux-gnu/libpython3.5.so")
# set(PYTHON_INCLUDE_DIR "/usr/lib/python3.5")

include_directories(
        include
        ../utils/include
        ../driver/include
        ../pose_estimate/include
        ${PYTHON_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenVINO_INCLUDE_DIRS}
)

add_library(armor_detector
        src/Detector.cpp
        src/LightBarFinder.cpp
        src/ArmorFinder.cpp
        src/Classifier.cpp
        src/CNN.cpp
        src/Score.cpp
        # src/buff_detector.cpp
        src/Inference.cpp
        src/ROI_Accelerator.cpp
        src/TargetChooser.cpp
        src/NumberClassifier.cpp
        src/GreenLightDetector.cpp
        src/YOLOv10Detector.cpp
        )

target_link_libraries(armor_detector
        utils
        pose_estimate
        -lpthread
        ${PYTHON_LIBRARIES}
        ${OpenCV_LIBS}
        openvino::runtime
        # ${NGRAPH_LIBRARIES}
        ${OpenVINO_LIBRARIES}
        ${OpenVINO_INCLUDE_DIRS}
        ${Eigen3_LIBRARIES}
)
