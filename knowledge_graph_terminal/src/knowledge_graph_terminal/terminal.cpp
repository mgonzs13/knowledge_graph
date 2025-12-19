// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <csignal>
#include <cstring>
#include <list>
#include <map>
#include <memory>
#include <readline/history.h>
#include <readline/readline.h>
#include <regex>
#include <set>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/graph.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"
#include "knowledge_graph_terminal/terminal.hpp"

namespace knowledge_graph_terminal {

static knowledge_graph::KnowledgeGraph *current_graph = nullptr;

std::vector<std::string> tokenize(const std::string &text,
                                  const std::string delim) {
  if (text.empty()) {
    return {};
  }

  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos) {
    end = text.find(delim, start);
    ret.push_back(text.substr(
        start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  return ret;
}

void print_properties(
    std::ostringstream &os,
    const std::vector<knowledge_graph_msgs::msg::Property> &props) {
  if (props.empty())
    return;

  os << "Properties:" << std::endl;
  for (const auto &prop : props) {
    os << "  " << prop.key << ": ";
    const auto &content = prop.value;
    switch (content.type) {
    case knowledge_graph_msgs::msg::Content::BOOL:
      os << (content.bool_value ? "true" : "false");
      break;
    case knowledge_graph_msgs::msg::Content::INT:
      os << content.int_value;
      break;
    case knowledge_graph_msgs::msg::Content::FLOAT:
      os << content.float_value;
      break;
    case knowledge_graph_msgs::msg::Content::DOUBLE:
      os << content.double_value;
      break;
    case knowledge_graph_msgs::msg::Content::STRING:
      os << "\"" << content.string_value << "\"";
      break;
    case knowledge_graph_msgs::msg::Content::VBOOL:
      os << "[";
      for (size_t i = 0; i < content.bool_vector.size(); ++i) {
        if (i > 0)
          os << ", ";
        os << (content.bool_vector[i] ? "true" : "false");
      }
      os << "]";
      break;
    case knowledge_graph_msgs::msg::Content::VINT:
      os << "[";
      for (size_t i = 0; i < content.int_vector.size(); ++i) {
        if (i > 0)
          os << ", ";
        os << content.int_vector[i];
      }
      os << "]";
      break;
    case knowledge_graph_msgs::msg::Content::VFLOAT:
      os << "[";
      for (size_t i = 0; i < content.float_vector.size(); ++i) {
        if (i > 0)
          os << ", ";
        os << content.float_vector[i];
      }
      os << "]";
      break;
    case knowledge_graph_msgs::msg::Content::VDOUBLE:
      os << "[";
      for (size_t i = 0; i < content.double_vector.size(); ++i) {
        if (i > 0)
          os << ", ";
        os << content.double_vector[i];
      }
      os << "]";
      break;
    case knowledge_graph_msgs::msg::Content::VSTRING:
      os << "[";
      for (size_t i = 0; i < content.string_vector.size(); ++i) {
        if (i > 0)
          os << ", ";
        os << "\"" << content.string_vector[i] << "\"";
      }
      os << "]";
      break;
    default:
      os << "unknown type";
    }
    os << std::endl;
  }
}

void pop_front(std::vector<std::string> &tokens) {
  if (!tokens.empty()) {
    tokens.erase(tokens.begin(), tokens.begin() + 1);
  }
}

// LCOV_EXCL_START
char *completion_generator(const char *text, int state) {
  // This function is called with state=0 the first time; subsequent calls are
  // with a nonzero state. state=0 can be used to perform one-time
  // initialization for this completion session.
  static std::vector<std::string> matches;
  static size_t match_index = 0;

  if (state == 0) {
    // During initialization, compute the actual matches for 'text' and keep
    // them in a static vector.
    matches.clear();
    match_index = 0;

    // Collect a vector of matches based on current context.
    std::string textstr = std::string(text);
    auto current_text = tokenize(rl_line_buffer);
    std::vector<std::string> candidates;

    if (current_text.size() <= 1) {
      candidates = {"add", "remove", "get", "print"};
    } else if (current_text.size() == 2) {
      if (current_text[0] == "add") {
        candidates = {"node", "edge"};
      } else if (current_text[0] == "remove") {
        candidates = {"node", "edge"};
      } else if (current_text[0] == "get") {
        candidates = {"node", "edge", "nodes", "edges"};
      }
    } else if (current_text[0] == "get") {
      if (current_text[1] == "node" && current_text.size() == 3) {
        // complete node names
        if (current_graph) {
          const auto &nodes = current_graph->get_nodes();
          for (const auto &node : nodes) {
            candidates.push_back(node.get_name());
          }
        }
      } else if (current_text[1] == "edge") {
        if (current_text.size() == 3) {
          // complete edge types
          if (current_graph) {
            const auto &edges = current_graph->get_edges();
            std::set<std::string> types;
            for (const auto &edge : edges) {
              types.insert(edge.get_type());
            }
            candidates.assign(types.begin(), types.end());
          }
        } else if (current_text.size() == 4) {
          // complete sources for the given type
          if (current_graph) {
            const auto &edges = current_graph->get_edges();
            std::set<std::string> sources;
            for (const auto &edge : edges) {
              if (edge.get_type() == current_text[2]) {
                sources.insert(edge.get_source_node());
              }
            }
            candidates.assign(sources.begin(), sources.end());
          }
        } else if (current_text.size() == 5) {
          // complete targets for the given type and source
          if (current_graph) {
            const auto &edges = current_graph->get_edges();
            std::set<std::string> targets;
            for (const auto &edge : edges) {
              if (edge.get_type() == current_text[2] &&
                  edge.get_source_node() == current_text[3]) {
                targets.insert(edge.get_target_node());
              }
            }
            candidates.assign(targets.begin(), targets.end());
          }
        }
      }
    } else if (current_text[0] == "remove") {
      if (current_text[1] == "node" && current_text.size() == 3) {
        // complete node names
        if (current_graph) {
          const auto &nodes = current_graph->get_nodes();
          for (const auto &node : nodes) {
            candidates.push_back(node.get_name());
          }
        }
      } else if (current_text[1] == "edge") {
        if (current_text.size() == 3) {
          // complete edge types
          if (current_graph) {
            const auto &edges = current_graph->get_edges();
            std::set<std::string> types;
            for (const auto &edge : edges) {
              types.insert(edge.get_type());
            }
            candidates.assign(types.begin(), types.end());
          }
        } else if (current_text.size() == 4) {
          // complete sources for the given type
          if (current_graph) {
            const auto &edges = current_graph->get_edges();
            std::set<std::string> sources;
            for (const auto &edge : edges) {
              if (edge.get_type() == current_text[2]) {
                sources.insert(edge.get_source_node());
              }
            }
            candidates.assign(sources.begin(), sources.end());
          }
        } else if (current_text.size() == 5) {
          // complete targets for the given type and source
          if (current_graph) {
            const auto &edges = current_graph->get_edges();
            std::set<std::string> targets;
            for (const auto &edge : edges) {
              if (edge.get_type() == current_text[2] &&
                  edge.get_source_node() == current_text[3]) {
                targets.insert(edge.get_target_node());
              }
            }
            candidates.assign(targets.begin(), targets.end());
          }
        }
      }
    }

    for (auto word : candidates) {
      if (word.size() >= textstr.size() &&
          word.compare(0, textstr.size(), textstr) == 0) {
        matches.push_back(word);
      }
    }
  }

  if (match_index >= matches.size()) {
    // We return nullptr to notify the caller no more matches are available.
    return nullptr;
  } else {
    // Return a malloc'd char* for the match. The caller frees it.
    return strdup(matches[match_index++].c_str());
  }
}

char **completer(const char *text, int start, int end) {
  // Don't do filename completion even if our generator finds no matches.
  rl_attempted_completion_over = 1;

  // Note: returning nullptr here will make readline use the default filename
  // completer.

  return rl_completion_matches(text, completion_generator);
}

Terminal::Terminal()
    : rclcpp::Node("graph_terminal_" + std::to_string(static_cast<int>(
                                           rclcpp::Clock().now().seconds()))) {}

void Terminal::run_console() {
  this->graph_ = std::make_shared<knowledge_graph::KnowledgeGraph>(
      this->shared_from_this());
  std::string line;
  bool success = true;

  std::cout << "Knowledge Graph console. Type \"quit\" to finish" << std::endl;

  rl_attempted_completion_function = completer;
  current_graph = this->graph_.get();
  rl_catch_signals = 0;
  signal(SIGINT, SIG_IGN);

  bool finish = false;
  while (!finish) {
    char *line = readline("> ");

    if (line == NULL || (strcmp(line, "quit") == 0)) {
      finish = true;
    }

    if (!finish && line && strlen(line) > 0) {
      add_history(line);

      std::string line_str(line);
      free(line);

      this->clean_command(line_str);

      std::ostringstream os;
      this->process_command(line_str, os);
      std::cout << os.str();
    }
  }

  std::cout << "Finishing..." << std::endl;
  this->graph_ = nullptr;
}
// LCOV_EXCL_STOP

void Terminal::clean_command(std::string &command) {
  // remove continuous spaces
  size_t pos;
  while ((pos = command.find("  ")) != command.npos) {
    command.erase(pos, 1);
  }

  // remove from spaces
  while (*command.begin() == ' ') {
    command.erase(0, 1);
  }

  // remove trailing spaces
  while (command[command.size() - 1] == ' ') {
    command.pop_back();
  }
}

void Terminal::process_command(std::string &command, std::ostringstream &os) {
  std::vector<std::string> tokens = tokenize(command);

  if (tokens.empty()) {
    return;
  }

  if (tokens[0] == "add") {
    pop_front(tokens);
    this->process_add(tokens, os);
  } else if (tokens[0] == "remove") {
    pop_front(tokens);
    this->process_remove(tokens, os);
  } else if (tokens[0] == "get") {
    pop_front(tokens);
    this->process_get(tokens, os);
  } else if (tokens[0] == "print") {
    pop_front(tokens);
    this->process_print(tokens, os);
  } else {
    os << "Command not found" << std::endl;
  }
}

void Terminal::process_add(std::vector<std::string> &command,
                           std::ostringstream &os) {
  if (command.size() > 0) {
    if (command[0] == "node") {
      pop_front(command);
      this->process_add_node(command, os);
    } else if (command[0] == "edge") {
      pop_front(command);
      this->process_add_edge(command, os);
    } else {
      os << "\tUsage: \n\t\tadd [node|edge]..." << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tadd [node|edge]..." << std::endl;
  }
}

void Terminal::process_add_node(std::vector<std::string> &command,
                                std::ostringstream &os) {
  if (command.size() == 2) {
    if (!this->graph_->has_node(command[0])) {
      this->graph_->create_node(command[0], command[1]);
    }
  } else {
    os << "\tUsage: \n\t\tadd node [name] [type]" << std::endl;
  }
}

void Terminal::process_add_edge(std::vector<std::string> &command,
                                std::ostringstream &os) {
  if (command.size() >= 3) {

    const std::string &edge_class = command[0];
    const std::string &source = command[1];
    const std::string &target = command[2];

    if (!this->graph_->has_edge(edge_class, source, target)) {
      this->graph_->create_edge(edge_class, source, target);
    } else {
      os << "Could not add the edge [" << command[0] << "] " << command[1]
         << " -> " << command[2] << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tadd edge [class] [source] [target]" << std::endl;
  }
}

void Terminal::process_remove(std::vector<std::string> &command,
                              std::ostringstream &os) {
  if (command.size() > 0) {
    if (command[0] == "node") {
      pop_front(command);
      this->process_remove_node(command, os);
    } else if (command[0] == "edge") {
      pop_front(command);
      this->process_remove_edge(command, os);
    } else {
      os << "\tUsage: \n\t\tremove [node|edge]..." << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove [node|edge]..." << std::endl;
  }
}

void Terminal::process_remove_node(std::vector<std::string> &command,
                                   std::ostringstream &os) {
  if (command.size() == 1) {
    if (this->graph_->has_node(command[0])) {
      auto node = this->graph_->get_node(command[0]);
      if (!this->graph_->remove_node(node)) {
        os << "Could not remove the node [" << command[0] << "]" << std::endl;
      }
    }
  } else {
    os << "\tUsage: \n\t\tremove node [name] [type]" << std::endl;
  }
}

void Terminal::process_remove_edge(std::vector<std::string> &command,
                                   std::ostringstream &os) {
  if (command.size() >= 3) {

    const std::string &edge_class = command[0];
    const std::string &source = command[1];
    const std::string &target = command[2];

    if (this->graph_->has_edge(edge_class, source, target)) {
      auto edge = this->graph_->get_edge(edge_class, source, target);
      if (!this->graph_->remove_edge(edge)) {
        os << "Could not remove the edge [" << command[0] << "] " << command[1]
           << " -> " << command[2] << std::endl;
      }
    }
  } else {
    os << "\tUsage: \n\t\tremove edge [class] [source] [target]" << std::endl;
  }
}

void Terminal::process_get(std::vector<std::string> &command,
                           std::ostringstream &os) {
  if (command.size() > 0) {
    if (command[0] == "node") {
      pop_front(command);
      this->process_get_node(command, os);
    } else if (command[0] == "edge") {
      pop_front(command);
      this->process_get_edge(command, os);
    } else if (command[0] == "nodes") {
      pop_front(command);
      this->process_get_nodes(command, os);
    } else if (command[0] == "edges") {
      pop_front(command);
      this->process_get_edges(command, os);
    } else {
      os << "\tUsage: \n\t\tremove [node|edge]..." << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove [node|edge]..." << std::endl;
  }
}

void Terminal::process_get_node(std::vector<std::string> &command,
                                std::ostringstream &os) {
  if (command.size() == 1) {
    if (this->graph_->has_node(command[0])) {
      auto node = this->graph_->get_node(command[0]);
      os << node.to_string() << std::endl;
      print_properties(os, node.properties_to_msg());
    } else {
      os << "node [" << command[0] << "] not found" << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget node [node_name]" << std::endl;
  }
}

void Terminal::process_get_edge(std::vector<std::string> &command,
                                std::ostringstream &os) {
  if (command.size() == 3) {

    if (this->graph_->has_edge(command[0], command[1], command[2])) {
      auto edge = this->graph_->get_edge(command[0], command[1], command[2]);
      os << edge.to_string() << std::endl;
      print_properties(os, edge.properties_to_msg());
    } else {
      os << "edge [" << command[0] << "] " << command[1] << " -> " << command[2]
         << " not found" << std::endl;
    }

  } else {
    os << "\tUsage: \n\t\tget edge [type] [source] [target]" << std::endl;
  }
}

void Terminal::process_get_nodes(std::vector<std::string> &command,
                                 std::ostringstream &os) {
  if (command.size() == 0) {
    const auto &nodes = this->graph_->get_nodes();

    os << "Nodes: " << nodes.size() << std::endl;
    for (const auto &node : nodes) {
      os << node.to_string() << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget nodes" << std::endl;
  }
}

void Terminal::process_get_edges(std::vector<std::string> &command,
                                 std::ostringstream &os) {
  if (command.size() == 0) {
    const auto &edges = this->graph_->get_edges();

    os << "Connections: " << edges.size() << std::endl;
    for (const auto &edge : edges) {
      os << edge.to_string() << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget edges" << std::endl;
  }
}

void Terminal::process_print(std::vector<std::string> &command,
                             std::ostringstream &os) {
  const auto &nodes = this->graph_->get_nodes();

  os << "Nodes: " << nodes.size() << std::endl;
  os << "==============" << std::endl;
  for (const auto &node : nodes) {
    os << "\t" << node.to_string() << std::endl;
  }

  const auto &edges = this->graph_->get_edges();

  os << std::endl;
  os << "Connections: " << edges.size() << std::endl;
  os << "==============" << std::endl;
  for (const auto &edge : edges) {
    os << "\t" << edge.to_string() << std::endl;
  }
}

} // namespace knowledge_graph_terminal
