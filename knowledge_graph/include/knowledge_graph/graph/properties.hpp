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

#ifndef KNOWLEDGE_GRAPH__GRAPH__PROPERTIES_HPP_
#define KNOWLEDGE_GRAPH__GRAPH__PROPERTIES_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

namespace knowledge_graph {
namespace graph {

/**
 * @class Properties
 * @brief Class for managing a collection of properties with various data types.
 */
class Properties {

public:
  /**
   * @brief Default constructor.
   */
  Properties() = default;

  /**
   * @brief Constructor that initializes properties from a vector of Property
   * messages.
   * @param msg Vector of Property messages to initialize from.
   */
  Properties(const std::vector<knowledge_graph_msgs::msg::Property> &msg);

  /**
   * @brief Destructor.
   */
  ~Properties() = default;

  /**
   * @brief Check if a property with the given key exists.
   * @param key The key of the property.
   * @return True if the property exists, false otherwise.
   */
  bool has(const std::string &key) const;

  /**
   * @brief Get the type of the property with the given key.
   * @param key The key of the property.
   * @return The type of the property as a string.
   */
  std::string type(const std::string &key) const;

  /**
   * @brief Set the value of a property with the given key.
   * @tparam T The type of the property.
   * @param key The key of the property to set.
   * @param value The value to set.
   */
  template <typename T> void set(const std::string &key, const T &value) {

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

  /**
   * @brief Get the value of a property with the given key.
   * @tparam T The expected type of the property.
   * @param key The key of the property to retrieve.
   * @return The value of the property.
   */
  template <typename T> T get(const std::string &key) const {
    auto it = this->properties_.find(key);
    if (it != this->properties_.end()) {
      return *std::static_pointer_cast<T>(it->second);
    }
    throw std::runtime_error("Property not found: " + key);
  }

  /**
   * @brief Convert a property to a Property message.
   * @param key The key of the property to convert.
   * @return The Property message.
   */
  knowledge_graph_msgs::msg::Property to_msg(const std::string &key) const;

  /**
   * @brief Convert all properties to a vector of Property messages.
   * @return A vector of Property messages.
   */
  std::vector<knowledge_graph_msgs::msg::Property> to_msg() const;

private:
  /// @brief Internal storage for properties.
  std::unordered_map<std::string, std::shared_ptr<void>> properties_;
  /// @brief Registry to keep track of property types.
  std::unordered_map<std::string, std::string> registry_;
};

} // namespace graph
} // namespace knowledge_graph

#endif // KNOWLEDGE_GRAPH__GRAPH__PROPERTIES_HPP_