// Copyright 2025 Miguel Ángel González Santamarta
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

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "knowledge_graph/graph/properties.hpp"
#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

using namespace knowledge_graph::graph;

Properties::Properties(
    const std::vector<knowledge_graph_msgs::msg::Property> &msg) {
  for (const auto &prop_msg : msg) {
    const std::string &key = prop_msg.key;
    uint8_t type = prop_msg.value.type;

    if (type == knowledge_graph_msgs::msg::Content::BOOL) {
      this->set<bool>(key, prop_msg.value.bool_value);
    } else if (type == knowledge_graph_msgs::msg::Content::INT) {
      this->set<int>(key, prop_msg.value.int_value);
    } else if (type == knowledge_graph_msgs::msg::Content::FLOAT) {
      this->set<float>(key, prop_msg.value.float_value);
    } else if (type == knowledge_graph_msgs::msg::Content::DOUBLE) {
      this->set<double>(key, prop_msg.value.double_value);
    } else if (type == knowledge_graph_msgs::msg::Content::STRING) {
      this->set<std::string>(key, prop_msg.value.string_value);
    } else if (type == knowledge_graph_msgs::msg::Content::VBOOL) {
      this->set<std::vector<bool>>(key, prop_msg.value.bool_vector);
    } else if (type == knowledge_graph_msgs::msg::Content::VINT) {
      this->set<std::vector<int>>(key, prop_msg.value.int_vector);
    } else if (type == knowledge_graph_msgs::msg::Content::VFLOAT) {
      this->set<std::vector<float>>(key, prop_msg.value.float_vector);
    } else if (type == knowledge_graph_msgs::msg::Content::VDOUBLE) {
      this->set<std::vector<double>>(key, prop_msg.value.double_vector);
    } else if (type == knowledge_graph_msgs::msg::Content::VSTRING) {
      this->set<std::vector<std::string>>(key, prop_msg.value.string_vector);
    } else {
      throw std::runtime_error("Unsupported property type for key: " + key);
    }
  }
}

bool Properties::has(const std::string &key) const {
  return this->properties_.find(key) != this->properties_.end();
}

std::string Properties::type(const std::string &key) const {
  auto it = this->registry_.find(key);
  if (it != this->registry_.end()) {
    return it->second;
  }
  throw std::runtime_error("Property not found: " + key);
}

template <typename T>
void Properties::set(const std::string &key, const T &value) {

  if (typeid(T).name() != typeid(bool).name() &&
      typeid(T).name() != typeid(int).name() &&
      typeid(T).name() != typeid(float).name() &&
      typeid(T).name() != typeid(double).name() &&
      typeid(T).name() != typeid(std::string).name() &&
      typeid(T).name() != typeid(std::vector<bool>).name() &&
      typeid(T).name() != typeid(std::vector<int>).name() &&
      typeid(T).name() != typeid(std::vector<float>).name() &&
      typeid(T).name() != typeid(std::vector<double>).name() &&
      typeid(T).name() != typeid(std::vector<std::string>).name()) {
    throw std::runtime_error("Unsupported property type for key: " + key);
  }

  if (this->has(key)) {
    if (this->type(key) != typeid(T).name()) {
      throw std::runtime_error("Type mismatch for key: " + key);
    }

    *std::static_pointer_cast<T>(this->properties_[key]) = value;
  } else {
    this->properties_[key] = std::make_shared<T>(value);
    this->registry_[key] = typeid(T).name();
  }
}

template <typename T> T Properties::get(const std::string &key) const {
  auto it = this->properties_.find(key);
  if (it != this->properties_.end()) {
    return *std::static_pointer_cast<T>(it->second);
  }
  throw std::runtime_error("Property not found: " + key);
}

knowledge_graph_msgs::msg::Property
Properties::to_msg(const std::string &key) const {
  if (!this->has(key)) {
    throw std::runtime_error("Property not found: " + key);
  }

  knowledge_graph_msgs::msg::Property prop_msg;
  prop_msg.key = key;

  const std::string &type_name = this->type(key);
  if (type_name == typeid(bool).name()) {
    prop_msg.value.bool_value =
        *std::static_pointer_cast<bool>(this->properties_.at(key));
  } else if (type_name == typeid(int).name()) {
    prop_msg.value.int_value =
        *std::static_pointer_cast<int>(this->properties_.at(key));
  } else if (type_name == typeid(float).name()) {
    prop_msg.value.float_value =
        *std::static_pointer_cast<float>(this->properties_.at(key));
  } else if (type_name == typeid(double).name()) {
    prop_msg.value.double_value =
        *std::static_pointer_cast<double>(this->properties_.at(key));
  } else if (type_name == typeid(std::string).name()) {
    prop_msg.value.string_value =
        *std::static_pointer_cast<std::string>(this->properties_.at(key));
  } else if (type_name == typeid(std::vector<bool>).name()) {
    prop_msg.value.bool_vector =
        *std::static_pointer_cast<std::vector<bool>>(this->properties_.at(key));
  } else if (type_name == typeid(std::vector<int>).name()) {
    prop_msg.value.int_vector =
        *std::static_pointer_cast<std::vector<int>>(this->properties_.at(key));
  } else if (type_name == typeid(std::vector<float>).name()) {
    prop_msg.value.float_vector = *std::static_pointer_cast<std::vector<float>>(
        this->properties_.at(key));
  } else if (type_name == typeid(std::vector<double>).name()) {
    prop_msg.value.double_vector =
        *std::static_pointer_cast<std::vector<double>>(
            this->properties_.at(key));
  } else if (type_name == typeid(std::vector<std::string>).name()) {
    prop_msg.value.string_vector =
        *std::static_pointer_cast<std::vector<std::string>>(
            this->properties_.at(key));
  } else {
    throw std::runtime_error("Unsupported property type for key: " + key);
  }

  return prop_msg;
}

std::vector<knowledge_graph_msgs::msg::Property> Properties::to_msg() const {

  std::vector<knowledge_graph_msgs::msg::Property> msg_properties;

  for (const auto &pair : this->properties_) {
    const std::string &key = pair.first;
    knowledge_graph_msgs::msg::Property prop_msg = this->to_msg(key);
    msg_properties.push_back(prop_msg);
  }

  return msg_properties;
}
