cmake_minimum_required(VERSION 3.5)
project(knowledge_graph)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(knowledge_graph_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  knowledge_graph_msgs
)

include_directories(include)
install(DIRECTORY include/
  DESTINATION include/
)

add_library(${PROJECT_NAME} SHARED
  src/knowledge_graph/knowledge_graph.cpp
  src/knowledge_graph/graph_utils.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_python_install_package(${PROJECT_NAME})

ament_package()
