cmake_minimum_required(VERSION 3.5)
project(knowledge_graph_terminal)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(knowledge_graph REQUIRED)

include_directories(include)

add_library(graph_terminal SHARED
  src/knowledge_graph_terminal/terminal.cpp)
target_link_libraries(graph_terminal
  rclcpp::rclcpp
  knowledge_graph::knowledge_graph
)

add_executable(${PROJECT_NAME}
  src/terminal_node.cpp)
target_link_libraries(${PROJECT_NAME}
  graph_terminal
  rclcpp::rclcpp
  knowledge_graph::knowledge_graph
  readline
)

install(TARGETS
  graph_terminal
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
