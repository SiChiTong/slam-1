CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam)

# INCLUDE
INCLUDE_DIRECTORIES(
    slam_calibration/include
    slam_camera/include
    slam_estimation/include
    slam_feature/include
    slam_gimbal/include
    slam_kinematics/include
    slam_odometry/include
    slam_optimization/include
    slam_symmath/include
    slam_utils/include
)


# libslam_utils.a
ADD_SUBDIRECTORY(slam_utils)

# libslam_camera.a
ADD_SUBDIRECTORY(slam_camera)
ADD_DEPENDENCIES(slam_camera slam_utils)

# libslam_calibration.a
ADD_SUBDIRECTORY(slam_calibration)
ADD_DEPENDENCIES(slam_calibration slam_utils slam_camera)

# libslam_feature.a
ADD_SUBDIRECTORY(slam_feature)
ADD_DEPENDENCIES(slam_feature slam_utils slam_camera)

# libslam_optimization.a
ADD_SUBDIRECTORY(slam_optimization)
ADD_DEPENDENCIES(slam_optimization slam_utils)

# libslam_odometry.a
ADD_SUBDIRECTORY(slam_odometry)
ADD_DEPENDENCIES(
    slam_odometry
    slam_utils
    slam_camera
    slam_feature
    slam_optimization
)

# libslam_kinematics.a
ADD_SUBDIRECTORY(slam_kinematics)
ADD_DEPENDENCIES(slam_kinematics slam_utils)

# libslam_estimation.a
ADD_SUBDIRECTORY(slam_estimation)
ADD_DEPENDENCIES(slam_estimation slam_utils slam_kinematics)

# libslam_symath.a
ADD_SUBDIRECTORY(slam_symmath)
ADD_DEPENDENCIES(slam_symmath slam_utils)

# libslam_gimbal.a
ADD_SUBDIRECTORY(slam_gimbal)
ADD_DEPENDENCIES(slam_gimbal slam_utils)


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
