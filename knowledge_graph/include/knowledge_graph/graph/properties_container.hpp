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

#ifndef KNOWLEDGE__GRAPH__PROPERTIES_CONTAINER_HPP_
#define KNOWLEDGE__GRAPH__PROPERTIES_CONTAINER_HPP_

#include <string>

#include "knowledge_graph/graph/properties.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

namespace knowledge_graph {
namespace graph {

/**
 * @class PropertiesContainer
 * @brief Class that contains a Properties object and provides methods to
 * manage properties.
 */
class PropertiesContainer {
public:
  /**
   * @brief Default constructor.
   */
  PropertiesContainer() = default;

  /**
   * @brief Constructor that initializes the PropertiesContainer from a vector
   * of Property messages.
   * @param msg Vector of Property messages to initialize from.
   */
  PropertiesContainer(
      const std::vector<knowledge_graph_msgs::msg::Property> &msg);

  /**
   * @brief Destructor.
   */
  ~PropertiesContainer() = default;

  /**
   * @brief Set the value of a property with the given key.
   * @tparam T The type of the property.
   * @param key The key of the property to set.
   * @param value The value to set.
   */
  template <typename T>
  void set_property(const std::string &key, const T &value) {
    this->properties_.set<T>(key, value);
  }

  /**
   * @brief Get the value of a property with the given key.
   * @tparam T The type of the property.
   * @param key The key of the property to get.
   * @return The value of the property.
   */
  template <typename T> T get_property(const std::string &key) const {
    return this->properties_.get<T>(key);
  }

  /**
   * @brief Check if a property with the given key exists.
   * @param key The key of the property.
   * @return True if the property exists, false otherwise.
   */
  bool has_property(const std::string &key) const;

  /**
   * @brief Convert properties to a vector of Property messages.
   * @return Vector of Property messages.
   */
  std::vector<knowledge_graph_msgs::msg::Property> properties_to_msg() const;

private:
  /// @brief Properties object that holds the properties.
  Properties properties_;
};

} // namespace graph
} // namespace knowledge_graph

#endif // KNOWLEDGE__GRAPH__PROPERTIES_CONTAINER_HPP_