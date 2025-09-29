cmake_minimum_required(VERSION 3.8)
project(cablerobot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Collect all msg and srv files in the directories
file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
     "msg/*.msg")
file(GLOB srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
     "srv/*.srv")

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_package()
