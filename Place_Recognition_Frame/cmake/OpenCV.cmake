
set(OpenCV_DIR "/usr/include/opencv4/opencv2")

find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS})
# list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBS})
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBRARIES})