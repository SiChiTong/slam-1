CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_optimization)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# PACKAGES
FIND_PACKAGE(GTest REQUIRED)


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
    src/ba.cpp
    src/benchmark.cpp
    src/gd.cpp
    src/gnm.cpp
    src/lls.cpp
    src/nm.cpp
    src/ransac.cpp
)
SET(
    DEPS
    slam_optimization
    slam_utils
    ${GTEST_LIBRARIES}
    pthread
)



# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY scripts DESTINATION ${PROJECT_BINARY_DIR})

ADD_EXECUTABLE(gd_test tests/gd_test.cpp)
TARGET_LINK_LIBRARIES(gd_test ${DEPS})

ADD_EXECUTABLE(gnm_test tests/gnm_test.cpp)
TARGET_LINK_LIBRARIES(gnm_test ${DEPS})

ADD_EXECUTABLE(lls_test tests/lls_test.cpp)
TARGET_LINK_LIBRARIES(lls_test ${DEPS})

ADD_EXECUTABLE(nm_test tests/nm_test.cpp)
TARGET_LINK_LIBRARIES(nm_test ${DEPS})

ADD_EXECUTABLE(ransac_test tests/ransac_test.cpp)
TARGET_LINK_LIBRARIES(ransac_test ${DEPS})
