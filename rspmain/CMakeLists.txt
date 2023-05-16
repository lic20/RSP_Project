cmake_minimum_required(VERSION 3.8)
project(rspmain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pnpm REQUIRED)
find_package(base2tag REQUIRED)
find_package(rsp_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)

include_directories( include )

add_library( rspmain SHARED src/rspmain.cpp )
ament_target_dependencies( rspmain rclcpp rclcpp_action pnpm
                            base2tag rsp_msgs open_manipulator_msgs)

add_executable( main src/mainexec.cpp )
target_link_libraries( main rspmain )
ament_target_dependencies( main rclcpp rclcpp_action pnpm base2tag rsp_msgs open_manipulator_msgs)

install( DIRECTORY include/ DESTINATION include )
install( TARGETS rspmain LIBRARY DESTINATION lib )
install( TARGETS main DESTINATION lib/${PROJECT_NAME} )

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
