cmake_minimum_required(VERSION 3.8)
project(base2tag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(rsp_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)

include_directories( include )
include_directories( /usr/include/eigen3/Eigen)

add_library( base2tag_srv SHARED src/base2tag_srv.cpp )
ament_target_dependencies( base2tag_srv rclcpp open_manipulator_msgs
                            rsp_msgs geometry_msgs Eigen3)

add_executable( base2tag_server src/server_node.cpp )
target_link_libraries( base2tag_server base2tag_srv)
ament_target_dependencies( base2tag_server rclcpp open_manipulator_msgs rsp_msgs geometry_msgs Eigen3 )

install( DIRECTORY include/ DESTINATION include )
install( TARGETS base2tag_srv LIBRARY DESTINATION lib )
install( TARGETS base2tag_server DESTINATION lib/${PROJECT_NAME} )

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
