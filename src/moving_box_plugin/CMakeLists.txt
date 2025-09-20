cmake_minimum_required(VERSION 3.8)
project(moving_box_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)


include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})


add_library(${PROJECT_NAME} SHARED src/moving_box_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES})


add_library(moving_box2_plugin SHARED src/moving_box2_plugin.cpp)
ament_target_dependencies(moving_box2_plugin rclcpp)
target_link_libraries(moving_box2_plugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES})


install(TARGETS ${PROJECT_NAME} moving_box2_plugin 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

