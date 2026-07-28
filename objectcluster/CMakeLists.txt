cmake_minimum_required(VERSION 3.0.2)
project(objectcluster)


IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_CXX_STANDARD 17) 
set (CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -lpthread")


find_package(catkin REQUIRED COMPONENTS
        roscpp#
        rospy
        eigen_conversions #
        cv_bridge   #
        sensor_msgs#
        image_transport#
        nav_msgs#
        customized_msgs
        tf
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED QUIET)

catkin_package(

)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
)

link_directories(
        include
        ${OpenCV_LIBRARY_DIRS}
        ${PCL_LIBRARY_DIRS}
)

file(GLOB_RECURSE ALL_SRC "src/*.cc")
file(GLOB_RECURSE ALL_SRC_CPP "src/*.cpp")

add_executable(objseg_gridclass node/objseg_grid_class.cpp ${ALL_SRC} ${ALL_SRC_CPP})
add_dependencies(objseg_gridclass ${catkin_EXPORTED_TARGETS})
target_link_libraries(objseg_gridclass ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS 
    objseg_gridclass
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
# install(DIRECTORY
#         rviz/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
#     FILES_MATCHING PATTERN "*.rviz"
# )

