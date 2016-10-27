CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_symmath)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include/
    /usr/local/include
)


# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
)


# LIBRARY
ADD_LIBRARY(
    slam_symmath
    STATIC
    src/symmath.cpp
)
SET(
    DEPS
    slam_symmath
    ginac
)
