cmake_minimum_required(VERSION 3.5)
project(knowledge_graph_terminal)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(knowledge_graph REQUIRED)

set(dependencies
  rclcpp
  knowledge_graph
)

include_directories(include)

add_library(graph_terminal SHARED
  src/knowledge_graph_terminal/terminal.cpp)
ament_target_dependencies(graph_terminal ${dependencies})

add_executable(${PROJECT_NAME}
  src/terminal_node.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} graph_terminal readline)

install(TARGETS
  graph_terminal
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
