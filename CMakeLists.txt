CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam)


# libslam_utils.a
INCLUDE_DIRECTORIES(slam_utils/include)
ADD_SUBDIRECTORY(slam_utils)

# libslam_camera.a
INCLUDE_DIRECTORIES(slam_camera/include)
ADD_SUBDIRECTORY(slam_camera)
ADD_DEPENDENCIES(slam_camera slam_utils)

# libslam_calibration.a
INCLUDE_DIRECTORIES(slam_calibration/include)
ADD_SUBDIRECTORY(slam_calibration)
ADD_DEPENDENCIES(slam_calibration slam_utils slam_camera)

# libslam_feature.a
INCLUDE_DIRECTORIES(slam_feature/include)
ADD_SUBDIRECTORY(slam_feature)
ADD_DEPENDENCIES(slam_feature slam_utils slam_camera)

# libslam_odometry.a
INCLUDE_DIRECTORIES(slam_odometry/include)
ADD_SUBDIRECTORY(slam_odometry)
ADD_DEPENDENCIES(slam_odometry slam_utils slam_camera slam_feature)

# libslam_kinematics.a
INCLUDE_DIRECTORIES(slam_kinematics/include)
ADD_SUBDIRECTORY(slam_kinematics)
ADD_DEPENDENCIES(slam_kinematics slam_utils)

# libslam_estimation.a
INCLUDE_DIRECTORIES(slam_estimation/include)
ADD_SUBDIRECTORY(slam_estimation)
ADD_DEPENDENCIES(slam_estimation slam_utils slam_kinematics)

# libslam_symath.a
INCLUDE_DIRECTORIES(slam_symmath/include)
ADD_SUBDIRECTORY(slam_symmath)
ADD_DEPENDENCIES(slam_symmath slam_utils)


# libslam.a
ADD_CUSTOM_TARGET(
    libslam ALL
    COMMAND
        ${CMAKE_AR} rc libslam.a
            ${CMAKE_BINARY_DIR}/slam_utils/libslam_utils.a
            ${CMAKE_BINARY_DIR}/slam_camera/libslam_camera.a
            ${CMAKE_BINARY_DIR}/slam_calibration/libslam_calibration.a
            ${CMAKE_BINARY_DIR}/slam_feature/libslam_feature.a
            ${CMAKE_BINARY_DIR}/slam_odometry/libslam_odometry.a
            ${CMAKE_BINARY_DIR}/slam_kinematics/libslam_kinematics.a
            ${CMAKE_BINARY_DIR}/slam_estimation/libslam_estimation.a
            ${CMAKE_BINARY_DIR}/slam_symmath/libslam_symmath.a
    DEPENDS
        slam_utils
        slam_camera
        slam_calibration
        slam_feature
        slam_odometry
        slam_kinematics
        slam_estimation
        slam_symmath
)
