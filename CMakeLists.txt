cmake_minimum_required(VERSION 3.5)
project(lidar_perception)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)

# find ros dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

#set include paths
include_directories(
  ${PCL_INCLUDE_DIRS}
  src/include
)
#PCL suggested cmake commands
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set sources
set(SOURCES src/main.cpp src/minimal_subscriber.cpp src/include/minimal_subscriber.hpp src/include/PointXYZVI.hpp) 
set(POINTFILTER_SOURCES src/PointFilter.cpp src/include/PointXYZVI.hpp)
set(CLUSTER_SOURCES src/ClusterBuilder.cpp src/include/PointXYZVI.hpp)

#add_executable(talker src/publisher_member_function.cpp)
add_executable(pointfilter ${POINTFILTER_SOURCES})
add_executable(cluster ${CLUSTER_SOURCES})
add_executable(listener ${SOURCES}) 

target_link_libraries(pointfilter ${PCL_LIBRARIES})
target_link_libraries(cluster ${PCL_LIBRARIES})
target_link_libraries(listener ${PCL_LIBRARIES})

ament_target_dependencies(pointfilter rclcpp std_msgs pcl_conversions tf2 tf2_ros)
ament_target_dependencies(cluster rclcpp std_msgs pcl_conversions)
ament_target_dependencies(listener rclcpp sensor_msgs pcl_conversions)

install(TARGETS
  pointfilter
  cluster
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
