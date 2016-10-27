CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_utils)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include
    /usr/include/eigen3
    /usr/local/include
    include
)


# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
)


# LIBRARY
ADD_LIBRARY(slam_utils STATIC src/utils.cpp)
SET(DEPS slam_utils)


# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(utils_test tests/utils_test.cpp)
TARGET_LINK_LIBRARIES(utils_test ${DEPS})
