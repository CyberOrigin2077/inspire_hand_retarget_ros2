cmake_minimum_required(VERSION 3.8)
project(inspire_retarget)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
)

link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)
link_libraries(libboost_program_options.a fmt)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add the controller node executable
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node
  rclcpp
  std_msgs
)
target_link_libraries(controller_node
  ${Boost_LIBRARIES}
)

# Install the executable
install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
