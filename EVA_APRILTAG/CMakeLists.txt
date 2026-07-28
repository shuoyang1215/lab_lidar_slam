cmake_minimum_required(VERSION 3.5)
project(eva_april)
message("start make eva_april")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_CXX_STANDARD 17)

# set(OpenCV_DIR "/home/garry/tmp-libs/opencv-3.4.10/installed/share/OpenCV")


if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    add_definitions(-DON_ARM)
else ()
    set(ENABLE_X86_64 "")
endif ()


find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    MESSAGE("HAVE FOUNDED  OpenMP")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        pcl_ros
        pcl_conversions
        nav_msgs
        std_msgs
        sensor_msgs
        geometry_msgs
        customized_msgs
        message_filters
        cv_bridge
        )

set(ALL_TARGET_LIBRARIES "")
include(cmake/eigen.cmake)
include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/global_definition.cmake)
include(cmake/OpenCV.cmake)
include(cmake/boost.cmake)
include(cmake/geographic.cmake)
include(cmake/Sophus.cmake)
include(cmake/FindEigen.cmake)
include(cmake/ceres_solver.cmake)

catkin_package()

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "nodecpp/*.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "Thirdparty/*.cpp")
file(GLOB_RECURSE EVALUATION_SRCS "src/evo_apriltag/*.cpp")
file(GLOB_RECURSE ALL_SRCS_CC "src/*")

list(REMOVE_ITEM ALL_SRCS ${EVALUATION_SRCS})
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_library(eva_Lib ${ALL_SRCS_CC})

target_link_libraries(eva_Lib ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


#apriltag
add_executable(evaluation_node nodecpp/evaluation_node.cpp )
target_link_libraries(evaluation_node eva_Lib)

install(TARGETS 
    evaluation_node
    eva_Lib
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)