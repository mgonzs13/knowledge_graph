cmake_minimum_required(VERSION 3.5)
project(knowledge_graph_db)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(knowledge_graph REQUIRED)

install(PROGRAMS
  knowledge_graph_db/knowledge_graph_db_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME knowledge_graph_db_node
)

ament_package()
