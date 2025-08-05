cmake_minimum_required(VERSION 3.5)
project(autonomous_robot_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(keyboard_msgs REQUIRED)
find_package(keyboard REQUIRED)


set(dependencies
    rclcpp
    sensor_msgs
    geometry_msgs
    keyboard_msgs
    keyboard
)

# load launch-files, config, world
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY worlds/   DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/   DESTINATION share/${PROJECT_NAME}/models)

install(FILES
    gazebo_models_worlds_collection/worlds/small_city.world
    DESTINATION share/${PROJECT_NAME}/worlds
)

# load package.xml
install(FILES 
    package.xml
    DESTINATION share/${PROJECT_NAME}
)


# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(tb_key_control_node src/tb_key_control.cpp)
ament_target_dependencies(tb_key_control_node ${dependencies})

# Install Cpp executables
install(TARGETS
    tb_key_control_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
    scripts/occupancy_grid_pub.py
    scripts/spawn_entity.py
    scripts/maze_solver.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()