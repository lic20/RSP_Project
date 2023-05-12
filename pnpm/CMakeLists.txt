cmake_minimum_required(VERSION 3.8)
project(pnpm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rsp_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
# find_package(open_manipulator_x_controller REQUIRED)

include_directories( include )

add_library( pnpm SHARED src/pnpm.cpp )
ament_target_dependencies( pnpm rclcpp rclcpp_action open_manipulator_msgs
                            rsp_msgs sensor_msgs)

add_executable( server src/server.cpp )
target_link_libraries( server pnpm )
ament_target_dependencies( server rclcpp rclcpp_action rsp_msgs )

add_executable( client src/client.cpp )
target_link_libraries( client pnpm )
ament_target_dependencies( client rclcpp rclcpp_action rsp_msgs )

install( DIRECTORY include/ DESTINATION include )
install( TARGETS pnpm LIBRARY DESTINATION lib )
install( TARGETS server client DESTINATION lib/${PROJECT_NAME} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
