CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

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

# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
    /usr/local/opencv3/lib
)

# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/local/include
    /usr/local/opencv3/include
    /usr/include/eigen3
    ./include
)

# LIBRARY
ADD_LIBRARY(
    slam
    src/calibration.cpp
    src/camera.cpp
    src/feature.cpp
    src/estimation.cpp
    src/matcher.cpp
    src/model.cpp
    src/odometry.cpp
    src/symmath.cpp
    src/util.cpp
)
SET(
    DEPS
    slam
    ginac
    ${OPENCV3_LIBS}
)


# SETTINGS
SET(BIN_DEST ${CMAKE_BINARY_DIR}/bin)
SET(TEST_DEST ${CMAKE_BINARY_DIR}/tests)


# UNIT TESTS
FILE(COPY tests/test_data DESTINATION ${TEST_DEST})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)

ADD_EXECUTABLE(camera_test tests/camera_test.cpp)
TARGET_LINK_LIBRARIES(camera_test ${DEPS})

ADD_EXECUTABLE(estimation_test tests/estimation_test.cpp)
TARGET_LINK_LIBRARIES(estimation_test ${DEPS})

ADD_EXECUTABLE(odometry_test tests/odometry_test.cpp)
TARGET_LINK_LIBRARIES(odometry_test ${DEPS})

ADD_EXECUTABLE(symmath_test tests/symmath_test.cpp)
TARGET_LINK_LIBRARIES(symmath_test ${DEPS})


# UTILS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/utils)

ADD_EXECUTABLE(calibration utils/calibration.cpp)
TARGET_LINK_LIBRARIES(calibration ${DEPS})

ADD_EXECUTABLE(feature_demo utils/feature_demo.cpp)
TARGET_LINK_LIBRARIES(feature_demo ${DEPS})

ADD_EXECUTABLE(optical_flow utils/optical_flow.cpp)
TARGET_LINK_LIBRARIES(optical_flow ${DEPS})

ADD_EXECUTABLE(video_capture utils/video_capture.cpp)
TARGET_LINK_LIBRARIES(video_capture ${DEPS})
