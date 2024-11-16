cmake_minimum_required(VERSION 3.5)
project(kdl_arm_dynamixel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  orocos_kdl_vendor
  kdl_parser
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
)

#########
# build #
#########
add_executable(main_real
  src/main_real.cpp
)
add_executable(main_rviz
  src/main_rviz.cpp
)

target_include_directories(main_rviz PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(main_rviz ${dependencies}
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  )
target_include_directories(main_real PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(main_real ${dependencies}
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  )

###########
# install #
###########
install(TARGETS
  main_real
  main_rviz
  DESTINATION lib/${PROJECT_NAME}
)
 

ament_package()
