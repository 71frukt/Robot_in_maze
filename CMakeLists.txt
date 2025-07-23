cmake_minimum_required(VERSION 3.5)
project(autonomous_tb3)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    sensor_msgs
    geometry_msgs
)

include_directories(include)

add_executable(tb_key_control_node src/tb_key_control.cpp)

ament_target_dependencies(tb_key_control_node ${dependencies})

install(TARGETS
    tb_key_control_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch 
        DESTINATION 
    share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()