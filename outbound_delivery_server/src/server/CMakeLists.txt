cmake_minimum_required(VERSION 3.8)
project(server)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Crow REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(outbound_delivery_robot_interfaces REQUIRED)
find_package(CURL REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
include_directories(/usr/include/cppconn)

add_executable(
  ${PROJECT_NAME}
  src/server.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${CURL_INCLUDE_DIRS}
)

ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
    std_msgs
    ament_index_cpp
    outbound_delivery_robot_interfaces
)

target_link_libraries(
  ${PROJECT_NAME}
  Crow::Crow
  yaml-cpp
  mysqlcppconn
  ${CURL_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    include
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
