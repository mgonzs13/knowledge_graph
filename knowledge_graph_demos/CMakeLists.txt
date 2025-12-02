cmake_minimum_required(VERSION 3.8)
project(knowledge_graph_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(knowledge_graph REQUIRED)
find_package(knowledge_graph_msgs REQUIRED)

# Create demo executable
add_executable(knowledge_graph_demo src/knowledge_graph_demo.cpp)
target_link_libraries(knowledge_graph_demo
  rclcpp::rclcpp
  knowledge_graph::knowledge_graph
  ${knowledge_graph_msgs_TARGETS}
)

install(TARGETS
  knowledge_graph_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()