CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_calibration)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include/
    /usr/include/eigen3
    /usr/local/include
    /usr/local/opencv3/include
)


# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
    /usr/local/opencv3/lib
)


# OPENCV3
SET(OPENCV3_PATH /usr/local/opencv3)
SET(
    OPENCV3_LIBS
    ${OPENCV3_PATH}/lib/libopencv_calib3d.so
    ${OPENCV3_PATH}/lib/libopencv_core.so
    ${OPENCV3_PATH}/lib/libopencv_features2d.so
    ${OPENCV3_PATH}/lib/libopencv_flann.so
    ${OPENCV3_PATH}/lib/libopencv_highgui.so
    ${OPENCV3_PATH}/lib/libopencv_imgcodecs.so
    ${OPENCV3_PATH}/lib/libopencv_imgproc.so
    ${OPENCV3_PATH}/lib/libopencv_ml.so
    ${OPENCV3_PATH}/lib/libopencv_objdetect.so
    ${OPENCV3_PATH}/lib/libopencv_photo.so
    ${OPENCV3_PATH}/lib/libopencv_shape.so
    ${OPENCV3_PATH}/lib/libopencv_stitching.so
    ${OPENCV3_PATH}/lib/libopencv_superres.so
    ${OPENCV3_PATH}/lib/libopencv_ts.a
    ${OPENCV3_PATH}/lib/libopencv_video.so
    ${OPENCV3_PATH}/lib/libopencv_videoio.so
    ${OPENCV3_PATH}/lib/libopencv_videostab.so
    ${OPENCV3_PATH}/lib/libopencv_viz.so
)


# LIBRARY
ADD_LIBRARY(
    slam_calibration
    STATIC
    src/calibration.cpp
)
SET(
    DEPS
    slam_calibration
    slam_camera
    slam_utils
    ${OPENCV3_LIBS}
)


# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(calibration_test tests/calibration_test.cpp)
TARGET_LINK_LIBRARIES(calibration_test ${DEPS})


# UTILS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/utils)

ADD_EXECUTABLE(calibration utils/calibration.cpp)
TARGET_LINK_LIBRARIES(calibration ${DEPS})
