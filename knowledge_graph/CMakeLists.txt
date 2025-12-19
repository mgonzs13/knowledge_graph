cmake_minimum_required(VERSION 3.5)
project(knowledge_graph)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(knowledge_graph_msgs REQUIRED)

include_directories(include)
install(DIRECTORY include/
  DESTINATION include/
)

add_library(${PROJECT_NAME} SHARED
  src/knowledge_graph/graph/properties.cpp
  src/knowledge_graph/graph/properties_container.cpp
  src/knowledge_graph/graph/node.cpp
  src/knowledge_graph/graph/edge.cpp
  src/knowledge_graph/graph/graph.cpp
  src/knowledge_graph/knowledge_graph.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  ${knowledge_graph_msgs_TARGETS}
)
install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp knowledge_graph_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # C++ Tests
  ament_add_gtest(test_properties test/test_properties.cpp)
  target_link_libraries(test_properties ${PROJECT_NAME})

  ament_add_gtest(test_node test/test_node.cpp)
  target_link_libraries(test_node ${PROJECT_NAME})

  ament_add_gtest(test_edge test/test_edge.cpp)
  target_link_libraries(test_edge ${PROJECT_NAME})

  ament_add_gtest(test_graph test/test_graph.cpp)
  target_link_libraries(test_graph ${PROJECT_NAME})

  # Python Tests
  ament_add_pytest_test(test_properties_py test/test_properties.py)
  ament_add_pytest_test(test_node_py test/test_node.py)
  ament_add_pytest_test(test_edge_py test/test_edge.py)
  ament_add_pytest_test(test_graph_py test/test_graph.py)
endif()

ament_package()
