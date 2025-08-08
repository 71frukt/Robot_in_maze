cmake_minimum_required(VERSION 3.5)
project(fwd_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (required for ROS 2 Humble)
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)  # Вместо find_package(gazebo)

# Add the plugin library
add_library(rover_drive_plugin SHARED
    src/rover_drive_plugin.cpp
)

# Set include directories (only for your headers)
target_include_directories(rover_drive_plugin PRIVATE
    include
)

# Link dependencies
ament_target_dependencies(rover_drive_plugin
    "rclcpp"
    "geometry_msgs"
    "gazebo_ros"
)

# Set library output name (optional, but clear)
set_target_properties(rover_drive_plugin PROPERTIES
    PREFIX "lib"
    SUFFIX ".so"
)

# Install the library
install(TARGETS rover_drive_plugin
    DESTINATION lib
)

# Install other files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY fwd_bot_model/
    DESTINATION share/${PROJECT_NAME}/fwd_bot_model
)

install(FILES
    package.xml
    DESTINATION share/${PROJECT_NAME}
)

ament_package()