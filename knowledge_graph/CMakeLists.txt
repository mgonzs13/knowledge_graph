cmake_minimum_required(VERSION 3.5)
project(knowledge_graph)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(knowledge_graph_msgs REQUIRED)

include_directories(include)
install(DIRECTORY include/
  DESTINATION include/
)

add_library(${PROJECT_NAME} SHARED
  src/knowledge_graph/knowledge_graph.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
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
ament_export_dependencies(rclcpp rclcpp_lifecycle knowledge_graph_msgs)
ament_python_install_package(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Graph utils tests
  ament_add_gtest(graph_utils_test test/graph_utils_test.cpp)
  target_include_directories(graph_utils_test PRIVATE include)
  target_link_libraries(graph_utils_test
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${knowledge_graph_msgs_TARGETS}
  )

  # Knowledge graph tests
  ament_add_gtest(knowledge_graph_test test/knowledge_graph_test.cpp)
  target_include_directories(knowledge_graph_test PRIVATE include)
  target_link_libraries(knowledge_graph_test
    ${PROJECT_NAME}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${knowledge_graph_msgs_TARGETS}
  )

  # Python tests
  ament_add_pytest_test(test_graph_utils_py test/test_graph_utils.py)
  ament_add_pytest_test(test_knowledge_graph_py test/test_knowledge_graph.py)
endif()

ament_package()
