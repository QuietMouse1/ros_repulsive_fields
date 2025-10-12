cmake_minimum_required(VERSION 2.8.3)
project(repulsive_fields)

find_package(catkin REQUIRED COMPONENTS roscpp roslib dynamic_reconfigure std_msgs sensor_msgs geometry_msgs tf pcl_conversions pcl_ros laser_geometry)
find_package(PCL 1.2 REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(rospy REQUIRED)

generate_dynamic_reconfigure_options(cfg/setParameters.cfg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(repulsive_fields src/repulsive_fields.cpp)
target_link_libraries(repulsive_fields ${catkin_LIBRARIES})
add_dependencies(repulsive_fields ${PROJECT_NAME}_gencfg)