cmake_minimum_required(VERSION 3.5)
project(fwd_bot)

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

# find dependencies
find_package( ament_cmake REQUIRED )
find_package( rclcpp      REQUIRED )
find_package( rclpy       REQUIRED )
find_package( gazebo      REQUIRED )

set(dependencies
    rclcpp
    geometry_msgs
)

install( DIRECTORY launch/         DESTINATION share/${PROJECT_NAME}/launch        )
install( DIRECTORY fwd_bot_model/  DESTINATION share/${PROJECT_NAME}/fwd_bot_model )

install(FILES 
    package.xml
    DESTINATION share/${PROJECT_NAME}
)

include_directories(${GAZEBO_INCLUDE_DIRS})

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(rover_drive_plugin src/tank_drive_plugin.cpp)
ament_target_dependencies(rover_drive_plugin ${dependencies})



if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    # set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    # set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()