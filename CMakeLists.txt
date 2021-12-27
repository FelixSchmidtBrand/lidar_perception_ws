cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

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
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)
find_package(Boost REQUIRED)

# find ros dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# add_definitions(${PCL_DEFINITIONS})

#set include paths
include_directories(
  ${PCL_INCLUDE_DIRS}
  src/include
)

set(SOURCES src/main.cpp src/minimal_subscriber.cpp src/include/minimal_subscriber.hpp)

add_executable(talker src/publisher_member_function.cpp)
add_executable(listener ${SOURCES}) 

target_link_libraries(talker ${PCL_LIBRARIES})
target_link_libraries(listener ${PCL_LIBRARIES}) 
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp sensor_msgs pcl_conversions)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
