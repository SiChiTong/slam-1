CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_optimization)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include/
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
ADD_LIBRARY(
    slam_optimization
    STATIC
    src/ransac.cpp
)
SET(
    DEPS
    slam_optimization
    slam_utils
)



# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY scripts DESTINATION ${PROJECT_BINARY_DIR})

ADD_EXECUTABLE(ransac_test tests/ransac_test.cpp)
TARGET_LINK_LIBRARIES(ransac_test ${DEPS})