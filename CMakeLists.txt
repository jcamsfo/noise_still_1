cmake_minimum_required(VERSION 3.0)
project(MyFFmpegOpenCVProject)

set(CMAKE_CXX_STANDARD 11)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVFORMAT libavformat REQUIRED)
pkg_check_modules(AVCODEC libavcodec REQUIRED)
pkg_check_modules(AVUTIL libavutil REQUIRED)
pkg_check_modules(SWSCALE libswscale REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${AVFORMAT_INCLUDE_DIRS}
                    ${AVCODEC_INCLUDE_DIRS}
                    ${AVUTIL_INCLUDE_DIRS}
                    ${SWSCALE_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

add_executable(my_ffmpeg_opencv_program main.cpp)

target_link_libraries(my_ffmpeg_opencv_program ${AVFORMAT_LIBRARIES}
                      ${AVCODEC_LIBRARIES}
                      ${AVUTIL_LIBRARIES}
                      ${SWSCALE_LIBRARIES}
                      ${OpenCV_LIBS})

                    #   const char* imageFile1 = "/home/jim/Desktop/PiTests/images/still1bw.jpg";