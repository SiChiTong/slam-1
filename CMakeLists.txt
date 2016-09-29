CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


# PREREQUISITS
FIND_PACKAGE(OpenCV REQUIRED)


# LIBS
LINK_DIRECTORIES(
    /usr/local/lib
    /usr/lib
)

# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/local/include
    /usr/include/eigen3
    ${OpenCV_INCLUDE_DIRS}
    ./include
)

# LIBRARY
ADD_LIBRARY(
    slam
    src/calibration.cpp
    src/camera.cpp
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
    ${OpenCV_LIBS}
    ginac
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

ADD_EXECUTABLE(video_capture utils/video_capture.cpp)
TARGET_LINK_LIBRARIES(video_capture ${DEPS})
