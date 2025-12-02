# knowledge_graph

Many times, when you are developing a Software Architecture for robots, you need shared memory where to store information. There are many strategies for this. One of them is a blackboard. Another approach is a graph in which we store elements as graph nodes and relations as graph edges.

ROS 2 knowledge graph provides you a way to share a graph between nodes running in an application. The graph is distributed in all the ROS 2 nodes. Each node contains a replica that is synchronized with all the other replicas, guaranteeing a Strong Eventual Consistency.

<div align="center">

[![License: Apache-2](https://img.shields.io/badge/GitHub-GPL--3.0-informational)](https://opensource.org/license/apache-2)
[![GitHub release](https://img.shields.io/github/release/mgonzs13/knowledge_graph.svg)](https://github.com/mgonzs13/knowledge_graph/releases)
[![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/knowledge_graph.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph?branch=main)
[![Dependencies](https://img.shields.io/librariesio/github/mgonzs13/knowledge_graph?branch=main)](https://libraries.io/github/mgonzs13/knowledge_graph?branch=main)
[![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/knowledge_graph.svg)](https://github.com/mgonzs13/knowledge_graph/commits/main)

[![GitHub issues](https://img.shields.io/github/issues/mgonzs13/knowledge_graph)](https://github.com/mgonzs13/knowledge_graph/issues)
[![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/knowledge_graph)](https://github.com/mgonzs13/knowledge_graph/pulls)
[![Contributors](https://img.shields.io/github/contributors/mgonzs13/knowledge_graph.svg)](https://github.com/mgonzs13/knowledge_graph/graphs/contributors)

[![Python Formatter Check](https://github.com/mgonzs13/knowledge_graph/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph/actions/workflows/python-formatter.yml?branch=main)
[![C++ Formatter Check](https://github.com/mgonzs13/knowledge_graph/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph/actions/workflows/cpp-formatter.yml?branch=main) [![Documentation Deployment](https://github.com/mgonzs13/knowledge_graph/actions/workflows/documentation-deployment.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin)

| ROS 2 Distro |                                                                                                             Build and Test                                                                                                             |
| :----------: | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|   **Foxy**   |        [![Foxy Build](https://github.com/mgonzs13/knowledge_graph/actions/workflows/foxy-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph/actions/workflows/foxy-build-test.yml?branch=main)         |
| **Galatic**  |  [![Galactic Build](https://github.com/mgonzs13/knowledge_graph/actions/workflows/galactic-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph/actions/workflows/galactic-build-test.yml?branch=main)   |
|  **Humble**  | [![Humble Build and Test](https://github.com/mgonzs13/knowledge_graph/actions/workflows/humble-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph/actions/workflows/humble-build-test.yml?branch=main) |
|   **Iron**   |        [![Iron Build](https://github.com/mgonzs13/knowledge_graph/actions/workflows/iron-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph/actions/workflows/iron-build-test.yml?branch=main)         |
|  **Jazzy**   |       [![Jazzy Build](https://github.com/mgonzs13/knowledge_graph/actions/workflows/jazzy-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph/actions/workflows/jazzy-build-test.yml?branch=main)       |
|  **Kilted**  |     [![Kilted Build](https://github.com/mgonzs13/knowledge_graph/actions/workflows/kilted-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph/actions/workflows/kilted-build-test.yml?branch=main)      |
| **Rolling**  |    [![Rolling Build](https://github.com/mgonzs13/knowledge_graph/actions/workflows/rolling-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/knowledge_graph/actions/workflows/rolling-build-test.yml?branch=main)    |

</div>

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/knowledge_graph.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Testing

The C++ knowledge graph includes unit tests using GTest. To run the tests:

```shell
cd ~/ros2_ws
colcon test --packages-select knowledge_graph
colcon test-result --verbose
```

## Usage

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "knowledge_graph/knowledge_graph.hpp"

class NodeA : public rclcpp::Node {
public:
  NodeA() : rclcpp::Node("node_a") {
    this->graph_ = KnowledgeGraph::get_instance(shared_from_this());
  }

private:
  std::shared_ptr<knowledge_graph::KnowledgeGraph> graph_;
};


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NodeA>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### Python

```python
import rclpy
from rclpy.node import Node
from knowledge_graph import KnowledgeGraph


class NodeA(Node):
  def __init__(self) -> None:
    super().__init__("node_a")
    graph = KnowledgeGraph.get_instance(self)


def main():
  rclpy.init()
  node = NodeA()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## API

### C++

The complete C++ API of `knowledge_graph::KnowledgeGraph` is at [knowledge_graph/include/knowledge_graph/knowledge_graph.hpp](knowledge_graph/include/knowledge_graph/knowledge_graph.hpp), but it is mainly:

- Nodes

```cpp
bool update_node(const knowledge_graph_msgs::msg::Node &node, bool sync = true);
```

```cpp
bool remove_node(const std::string &node, bool sync = true);
```

```cpp
bool exist_node(const std::string &node);
```

```cpp
std::optional<knowledge_graph_msgs::msg::Node> get_node(const std::string &node);
```

```cpp
std::optional<knowledge_graph_msgs::msg::Node> get_node(const std::string &node);
```

- Edges

```cpp
bool update_edge(const knowledge_graph_msgs::msg::Edge &edge, bool sync = true);
```

```cpp
bool remove_edge(const knowledge_graph_msgs::msg::Edge &edge, bool sync = true);
```

```cpp
std::vector<knowledge_graph_msgs::msg::Edge> get_edges(const std::string &source, const std::string &target)
```

```cpp
std::vector<knowledge_graph_msgs::msg::Edge> get_edges(const std::string &edge_class)
```

```cpp
std::vector<knowledge_graph_msgs::msg::Edge> get_out_edges(const std::string &source)
```

```cpp
std::vector<knowledge_graph_msgs::msg::Edge> get_in_edges(const std::string &target)
```

### Python

The complete Ptyhon API of `KnowledgeGraph` is at [knowledge_graph/knowledge_graph/knowledge_graph.py](knowledge_graph/knowledge_graph/knowledge_graph.py), but it is mainly:

- Nodes

```python
def update_node(self, node: Node, sync: bool = True) -> bool
```

```python
def remove_node(self, node: str, sync: bool = True) -> bool
```

```python
def exist_node(self, node: str) -> bool
```

```python
def get_node(self, node: str) -> Node
```

```python
def get_nodes(self) -> List[Node]:
```

- Edges

```python
def update_edge(self, edge: Edge, sync: bool = True) -> bool
```

```python
def remove_edge(self, edge: Edge, sync: bool = True) -> bool
```

```python
def get_edges(self, source: str = None, target: str = None, edge_class: str = None) -> List[Edge]:
```

```python
def get_out_edges(self, source: str) -> List[Edge]
```

```python
def get_in_edges(self, target: str) -> List[Edge]
```

## Demos

- Termial & Viewer

```shell
$ ros2 run knowledge_graph_terminal knowledge_graph_terminal
```

```shell
$ rqt
```

![](./docs/demo_terminal_viewer.gif)

- SQLite

```shell
$ ros2 run knowledge_graph_db knowledge_graph_db_node --ros-args -p db_file:=my_knowledge_graph.db
```

![](./docs/demo_db.gif)
