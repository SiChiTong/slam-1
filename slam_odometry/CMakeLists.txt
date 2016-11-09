CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_odometry)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include/
    /usr/include/eigen3
    /usr/local/include
    /usr/local/opencv3/include
    include
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
    slam_odometry
    STATIC
    src/odometry.cpp
)
SET(
    DEPS
    slam_odometry
    slam_optimization
    slam_feature
    slam_camera
    slam_utils
    yaml-cpp
    ${OPENCV3_LIBS}
)


# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(odometry_test tests/odometry_test.cpp)
TARGET_LINK_LIBRARIES(odometry_test ${DEPS})


# UTILS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/utils)
FILE(COPY utils/data DESTINATION ${PROJECT_BINARY_DIR}/utils)
FILE(COPY scripts DESTINATION ${PROJECT_BINARY_DIR})

ADD_EXECUTABLE(optical_flow utils/optical_flow.cpp)
TARGET_LINK_LIBRARIES(optical_flow ${DEPS})
