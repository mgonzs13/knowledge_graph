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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "knowledge_graph/graph_utils.hpp"
#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/edge.hpp"
#include "knowledge_graph_msgs/msg/node.hpp"
#include "knowledge_graph_msgs/msg/property.hpp"

namespace knowledge_graph {
namespace test {

// =============================================================================
// Node Creation Tests
// =============================================================================

class NodeCreationTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

TEST_F(NodeCreationTest, CreateNodeWithValidParameters) {
  auto node = new_node("robot1", "robot");

  EXPECT_EQ(node.node_name, "robot1");
  EXPECT_EQ(node.node_class, "robot");
  EXPECT_TRUE(node.properties.empty());
}

TEST_F(NodeCreationTest, CreateNodeWithEmptyName) {
  auto node = new_node("", "robot");

  EXPECT_EQ(node.node_name, "");
  EXPECT_EQ(node.node_class, "robot");
}

TEST_F(NodeCreationTest, CreateNodeWithEmptyClass) {
  auto node = new_node("robot1", "");

  EXPECT_EQ(node.node_name, "robot1");
  EXPECT_EQ(node.node_class, "");
}

// =============================================================================
// Edge Creation Tests
// =============================================================================

class EdgeCreationTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

TEST_F(EdgeCreationTest, CreateEdgeWithValidParameters) {
  auto edge = new_edge("connects", "node_a", "node_b");

  EXPECT_EQ(edge.edge_class, "connects");
  EXPECT_EQ(edge.source_node, "node_a");
  EXPECT_EQ(edge.target_node, "node_b");
  EXPECT_TRUE(edge.properties.empty());
}

TEST_F(EdgeCreationTest, CreateSelfReferencingEdge) {
  auto edge = new_edge("self_loop", "node_a", "node_a");

  EXPECT_EQ(edge.edge_class, "self_loop");
  EXPECT_EQ(edge.source_node, "node_a");
  EXPECT_EQ(edge.target_node, "node_a");
}

// =============================================================================
// Content Creation Tests
// =============================================================================

class ContentCreationTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

TEST_F(ContentCreationTest, CreateBoolContent) {
  auto content_true = new_content<bool>(true);
  auto content_false = new_content<bool>(false);

  EXPECT_EQ(content_true.type, knowledge_graph_msgs::msg::Content::BOOL);
  EXPECT_TRUE(content_true.bool_value);

  EXPECT_EQ(content_false.type, knowledge_graph_msgs::msg::Content::BOOL);
  EXPECT_FALSE(content_false.bool_value);
}

TEST_F(ContentCreationTest, CreateIntContent) {
  auto content = new_content<int>(42);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::INT);
  EXPECT_EQ(content.int_value, 42);
}

TEST_F(ContentCreationTest, CreateIntContentNegative) {
  auto content = new_content<int>(-100);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::INT);
  EXPECT_EQ(content.int_value, -100);
}

TEST_F(ContentCreationTest, CreateFloatContent) {
  auto content = new_content<float>(3.14f);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::FLOAT);
  EXPECT_FLOAT_EQ(content.float_value, 3.14f);
}

TEST_F(ContentCreationTest, CreateDoubleContent) {
  auto content = new_content<double>(3.14159265359);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::DOUBLE);
  EXPECT_DOUBLE_EQ(content.double_value, 3.14159265359);
}

TEST_F(ContentCreationTest, CreateStringContent) {
  auto content = new_content<std::string>("hello world");

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::STRING);
  EXPECT_EQ(content.string_value, "hello world");
}

TEST_F(ContentCreationTest, CreateEmptyStringContent) {
  auto content = new_content<std::string>("");

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::STRING);
  EXPECT_EQ(content.string_value, "");
}

TEST_F(ContentCreationTest, CreateBoolVectorContent) {
  std::vector<bool> values = {true, false, true};
  auto content = new_content<std::vector<bool>>(values);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::VBOOL);
  EXPECT_EQ(content.bool_vector.size(), 3u);
  EXPECT_TRUE(content.bool_vector[0]);
  EXPECT_FALSE(content.bool_vector[1]);
  EXPECT_TRUE(content.bool_vector[2]);
}

TEST_F(ContentCreationTest, CreateIntVectorContent) {
  std::vector<int> values = {1, 2, 3, 4, 5};
  auto content = new_content<std::vector<int>>(values);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::VINT);
  EXPECT_EQ(content.int_vector, values);
}

TEST_F(ContentCreationTest, CreateFloatVectorContent) {
  std::vector<float> values = {1.1f, 2.2f, 3.3f};
  auto content = new_content<std::vector<float>>(values);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::VFLOAT);
  EXPECT_EQ(content.float_vector.size(), 3u);
}

TEST_F(ContentCreationTest, CreateDoubleVectorContent) {
  std::vector<double> values = {1.1, 2.2, 3.3};
  auto content = new_content<std::vector<double>>(values);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::VDOUBLE);
  EXPECT_EQ(content.double_vector.size(), 3u);
}

TEST_F(ContentCreationTest, CreateStringVectorContent) {
  std::vector<std::string> values = {"a", "b", "c"};
  auto content = new_content<std::vector<std::string>>(values);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::VSTRING);
  EXPECT_EQ(content.string_vector, values);
}

TEST_F(ContentCreationTest, CreateEmptyVectorContent) {
  std::vector<int> empty_values;
  auto content = new_content<std::vector<int>>(empty_values);

  EXPECT_EQ(content.type, knowledge_graph_msgs::msg::Content::VINT);
  EXPECT_TRUE(content.int_vector.empty());
}

// =============================================================================
// Content Extraction Tests
// =============================================================================

class ContentExtractionTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

TEST_F(ContentExtractionTest, GetBoolContent) {
  auto content = new_content<bool>(true);
  auto result = get_content<bool>(content);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result.value());
}

TEST_F(ContentExtractionTest, GetIntContent) {
  auto content = new_content<int>(42);
  auto result = get_content<int>(content);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 42);
}

TEST_F(ContentExtractionTest, GetFloatContent) {
  auto content = new_content<float>(3.14f);
  auto result = get_content<float>(content);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result.value(), 3.14f);
}

TEST_F(ContentExtractionTest, GetDoubleContent) {
  auto content = new_content<double>(3.14159);
  auto result = get_content<double>(content);

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value(), 3.14159);
}

TEST_F(ContentExtractionTest, GetStringContent) {
  auto content = new_content<std::string>("test");
  auto result = get_content<std::string>(content);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), "test");
}

TEST_F(ContentExtractionTest, GetVectorContent) {
  std::vector<int> values = {1, 2, 3};
  auto content = new_content<std::vector<int>>(values);
  auto result = get_content<std::vector<int>>(content);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), values);
}

TEST_F(ContentExtractionTest, GetContentWithWrongType) {
  auto content = new_content<int>(42);

  // Try to extract as different types
  auto bool_result = get_content<bool>(content);
  auto string_result = get_content<std::string>(content);
  auto float_result = get_content<float>(content);

  EXPECT_FALSE(bool_result.has_value());
  EXPECT_FALSE(string_result.has_value());
  EXPECT_FALSE(float_result.has_value());
}

// =============================================================================
// Property Tests
// =============================================================================

class PropertyTest : public ::testing::Test {
protected:
  knowledge_graph_msgs::msg::Node node;
  knowledge_graph_msgs::msg::Edge edge;

  void SetUp() override {
    node = new_node("test_node", "test_class");
    edge = new_edge("test_edge", "source", "target");
  }
};

TEST_F(PropertyTest, AddPropertyToNode) {
  bool result = add_property<int>(node, "count", 10);

  EXPECT_TRUE(result);
  EXPECT_EQ(node.properties.size(), 1u);
  EXPECT_EQ(node.properties[0].key, "count");
}

TEST_F(PropertyTest, AddMultiplePropertiesToNode) {
  add_property<int>(node, "count", 10);
  add_property<std::string>(node, "name", "robot");
  add_property<double>(node, "speed", 1.5);

  EXPECT_EQ(node.properties.size(), 3u);
}

TEST_F(PropertyTest, UpdateExistingProperty) {
  add_property<int>(node, "count", 10);
  add_property<int>(node, "count", 20);

  EXPECT_EQ(node.properties.size(), 1u);

  auto value = get_property<int>(node, "count");
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(value.value(), 20);
}

TEST_F(PropertyTest, AddPropertyToEdge) {
  bool result = add_property<double>(edge, "weight", 0.5);

  EXPECT_TRUE(result);
  EXPECT_EQ(edge.properties.size(), 1u);
  EXPECT_EQ(edge.properties[0].key, "weight");
}

TEST_F(PropertyTest, GetPropertyFromNode) {
  add_property<std::string>(node, "label", "my_label");

  auto result = get_property<std::string>(node, "label");

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), "my_label");
}

TEST_F(PropertyTest, GetPropertyFromEdge) {
  add_property<float>(edge, "distance", 10.5f);

  auto result = get_property<float>(edge, "distance");

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result.value(), 10.5f);
}

TEST_F(PropertyTest, GetNonExistentProperty) {
  auto result = get_property<int>(node, "nonexistent");

  EXPECT_FALSE(result.has_value());
}

TEST_F(PropertyTest, GetPropertyWithWrongType) {
  add_property<int>(node, "count", 10);

  auto result = get_property<std::string>(node, "count");

  EXPECT_FALSE(result.has_value());
}

TEST_F(PropertyTest, GetPropertyType) {
  add_property<int>(node, "int_prop", 10);
  add_property<std::string>(node, "string_prop", "test");
  add_property<double>(node, "double_prop", 3.14);

  EXPECT_EQ(get_property_type(node, "int_prop"),
            knowledge_graph_msgs::msg::Content::INT);
  EXPECT_EQ(get_property_type(node, "string_prop"),
            knowledge_graph_msgs::msg::Content::STRING);
  EXPECT_EQ(get_property_type(node, "double_prop"),
            knowledge_graph_msgs::msg::Content::DOUBLE);
}

TEST_F(PropertyTest, GetPropertyTypeNonExistent) {
  auto type = get_property_type(node, "nonexistent");

  EXPECT_EQ(type, knowledge_graph_msgs::msg::Content::ERROR);
}

// =============================================================================
// String Conversion Tests
// =============================================================================

class ToStringTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

TEST_F(ToStringTest, TypeToString) {
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::BOOL), "bool");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::INT), "int");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::FLOAT), "float");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::DOUBLE), "double");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::STRING), "string");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::VBOOL), "bool[]");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::VINT), "int[]");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::VFLOAT), "float[]");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::VDOUBLE), "double[]");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::VSTRING), "string[]");
  EXPECT_EQ(to_string(knowledge_graph_msgs::msg::Content::ERROR), "error");
}

TEST_F(ToStringTest, BoolContentToString) {
  auto content_true = new_content<bool>(true);
  auto content_false = new_content<bool>(false);

  EXPECT_EQ(to_string(content_true), "true");
  EXPECT_EQ(to_string(content_false), "false");
}

TEST_F(ToStringTest, IntContentToString) {
  auto content = new_content<int>(42);

  EXPECT_EQ(to_string(content), "42");
}

TEST_F(ToStringTest, StringContentToString) {
  auto content = new_content<std::string>("hello");

  EXPECT_EQ(to_string(content), "hello");
}

TEST_F(ToStringTest, NodeToString) {
  auto node = new_node("robot1", "robot");

  std::string result = to_string(node);

  EXPECT_TRUE(result.find("robot1") != std::string::npos);
  EXPECT_TRUE(result.find("robot") != std::string::npos);
}

TEST_F(ToStringTest, NodeWithPropertiesToString) {
  auto node = new_node("robot1", "robot");
  add_property<int>(node, "speed", 10);

  std::string result = to_string(node);

  EXPECT_TRUE(result.find("robot1") != std::string::npos);
  EXPECT_TRUE(result.find("speed") != std::string::npos);
  EXPECT_TRUE(result.find("10") != std::string::npos);
}

TEST_F(ToStringTest, EdgeToString) {
  auto edge = new_edge("connects", "node_a", "node_b");

  std::string result = to_string(edge);

  EXPECT_TRUE(result.find("connects") != std::string::npos);
  EXPECT_TRUE(result.find("node_a") != std::string::npos);
  EXPECT_TRUE(result.find("node_b") != std::string::npos);
  EXPECT_TRUE(result.find("->") != std::string::npos);
}

// =============================================================================
// Type From String Tests
// =============================================================================

class TypeFromStringTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

TEST_F(TypeFromStringTest, ValidTypeStrings) {
  EXPECT_EQ(type_from_string("bool"), knowledge_graph_msgs::msg::Content::BOOL);
  EXPECT_EQ(type_from_string("int"), knowledge_graph_msgs::msg::Content::INT);
  EXPECT_EQ(type_from_string("float"),
            knowledge_graph_msgs::msg::Content::FLOAT);
  EXPECT_EQ(type_from_string("double"),
            knowledge_graph_msgs::msg::Content::DOUBLE);
  EXPECT_EQ(type_from_string("string"),
            knowledge_graph_msgs::msg::Content::STRING);
}

TEST_F(TypeFromStringTest, VectorTypeStrings) {
  EXPECT_EQ(type_from_string("bool[]"),
            knowledge_graph_msgs::msg::Content::VBOOL);
  EXPECT_EQ(type_from_string("int[]"),
            knowledge_graph_msgs::msg::Content::VINT);
  EXPECT_EQ(type_from_string("float[]"),
            knowledge_graph_msgs::msg::Content::VFLOAT);
  EXPECT_EQ(type_from_string("double[]"),
            knowledge_graph_msgs::msg::Content::VDOUBLE);
  EXPECT_EQ(type_from_string("string[]"),
            knowledge_graph_msgs::msg::Content::VSTRING);
}

TEST_F(TypeFromStringTest, InvalidTypeString) {
  EXPECT_EQ(type_from_string("invalid"),
            knowledge_graph_msgs::msg::Content::ERROR);
  EXPECT_EQ(type_from_string(""), knowledge_graph_msgs::msg::Content::ERROR);
  EXPECT_EQ(type_from_string("Bool"),
            knowledge_graph_msgs::msg::Content::ERROR);
}

} // namespace test
} // namespace knowledge_graph

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
