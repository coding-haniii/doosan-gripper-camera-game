cmake_minimum_required(VERSION 3.10.2)
project(dsr_description2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. Install directories (URDF, Meshes, Launch, etc. to share folder)
install(DIRECTORY gripper meshes urdf xacro config ros2_control DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# 2. Install Python scripts to make them executable nodes
install(PROGRAMS
  scripts/dsr_vision_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Environment hooks and ament package (Must be at the very bottom!)
ament_environment_hooks("dsr_description2_path.dsv.in")
ament_package()