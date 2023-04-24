cmake_minimum_required(VERSION 3.8)
project(aruco_pose_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( include )

add_library(tag_PE SHARED src/tag_pose_estimation.cpp )
ament_target_dependencies(tag_PE rclcpp sensor_msgs OpenCV cv_bridge)

add_executable(tag_pose_estimation src/tag_PE_node.cpp)
target_link_libraries(tag_pose_estimation ${OpenCV_LIBS} tag_PE)
ament_target_dependencies(tag_pose_estimation rclcpp)

install( DIRECTORY include/ DESTINATION include )
install(TARGETS tag_PE LIBRARY DESTINATION lib)
install( TARGETS tag_pose_estimation DESTINATION lib/${PROJECT_NAME})
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
