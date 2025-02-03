# find_package(OpenCV 4 REQUIRED)

set(CMAKE_PREFIX_PATH "/usr/local/include/opencv4/")
set(OpenCV_DIR /usr/local/include/opencv4/)
find_package(OpenCV 4 REQUIRED)



include_directories(
        include
        ../utils/include
        ../armor_detector/include
        ../driver/include
        ${OpenCV_INCLUDE_DIRS}
)


add_library(driver
        src/SerialPort.cpp
        src/VideoCapture.cpp
        src/GxCamera.cpp
        src/VideoSaver.cpp
        src/Picture.cpp
        src/DaHen.cpp
        src/Video.cpp
        src/hik.cpp
)

target_include_directories(driver PUBLIC hikSDK/include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(driver PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64/
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(driver PUBLIC hikSDK/lib/arm64)
  install(
    DIRECTORY hikSDK/lib/arm64/
    DESTINATION lib
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()


target_link_libraries(driver
        utils
        -lpthread
        -lboost_system
        # libgxiapi.so
        ${OpenCV_LIBS}
        FormatConversion
        MediaProcess
        MvCameraControl
        MVRender
        MvUsb3vTL
)