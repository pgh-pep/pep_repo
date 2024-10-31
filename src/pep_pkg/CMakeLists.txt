cmake_minimum_required(VERSION 3.8)
project(pep_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Include directories
include_directories(include)

install(
    DIRECTORY config include launch models scripts src pep_pkg
    DESTINATION share/${PROJECT_NAME}
)

# Create Cpp executables
add_executable(publisher src/test_publisher.cpp) # Need for each node
ament_target_dependencies(publisher rclcpp std_msgs) # Add both nodes and dependencies

# Install Cpp executables
install(TARGETS
    publisher
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
    scripts/test_subscriber.py # Add each python node
    scripts/camera_stream.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
