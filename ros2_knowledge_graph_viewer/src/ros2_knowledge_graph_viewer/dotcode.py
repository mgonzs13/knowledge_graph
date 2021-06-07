# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# this is a modified version of rx/rxgraph/src/rxgraph/dotcode.py

import re
import copy

from ros2_knowledge_graph_viewer import ros2_knowledge_graph2_impl
import math
import rclpy
import pydot
import tf2_py
import tf2_ros
import numpy
from ros2_knowledge_graph_msgs.msg import Content, Edge

try:
    unicode
    # we're on python2, or the "unicode" function has already been defined elsewhere
except NameError:
    unicode = str
    # we're on python3


def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.

    quat = [x, y, z, w]
    """
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = numpy.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = numpy.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = numpy.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class Ros2KnowledgeGraphDotcodeGenerator:

    def __init__(self):
        # self.listener_ = tf.TransformListener()
        pass

    def generate_dotgraph(
        self,
        ros2_knowledge_graphinst,
        dotcode_factory,
        orientation='LR',
        rank='same',  # None, same, min, max, source, sink
        ranksep=0.2,  # vertical distance between layers
        rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
        simplify=True,  # remove double edges
        quiet=False,
        unreachable=False):
        """
        See generate_dotcode
        """

        # create the graph
        # result = "digraph G {\n
        # rankdir=%(orientation)s;\n%(nodes_str)s\n%(edges_str)s}\n" % vars()
        dotgraph = dotcode_factory.get_graph(
            rank=rank,
            ranksep=ranksep,
            simplify=simplify,
            rankdir=orientation)

        for node in ros2_knowledge_graphinst.graph.nodes:
             dotcode_factory.add_node_to_graph(
                 dotgraph,
                 nodename=node.node_name,
                 nodelabel=node.node_name,
                 shape='ellipse',
                 url=node.node_name)

        for edge in ros2_knowledge_graphinst.graph.edges:
            label = ''
            if edge.content.type == Content.BOOL:
              label = '[bool] ' + str(edge.content.bool_value)
            if edge.content.type == Content.INT:
              label = '[int] ' + str(edge.content.int_value)
            if edge.content.type == Content.FLOAT:
              label = '[float] ' + str(edge.content.float_value)
            if edge.content.type == Content.DOUBLE:
              label = '[double] ' + str(edge.content.double_value)
            if edge.content.type == Content.STRING:
              label = '[string] ' + edge.content.string_value
            if edge.content.type == Content.VBOOL:
              label = '[bool[]] of ' + str(len(edge.content.bool_vector))
            if edge.content.type == Content.VINT:
              label = '[int[]] ' + str(len(edge.content.int_vector))
            if edge.content.type == Content.VFLOAT:
              label = '[float[]] of ' + str(len(edge.content.float_vector))
            if edge.content.type == Content.VDOUBLE:
              label = '[double[]] of ' + str(len(edge.content.double_vector))
            if edge.content.type == Content.VSTRING:
              label = '[string[]] ' + str(len(edge.content.string_vector))
            if edge.content.type == Content.POSE:
              label = '[pose] (' + (
                str(edge.content.pose_value.pose.position.x) + ', ' +
                str(edge.content.pose_value.pose.position.y) + ', ' +
                str(edge.content.pose_value.pose.position.z) + ')')
            if edge.content.type == Content.TF or edge.content.type == Content.STATICTF:
              x = edge.content.tf_value.transform.translation.x
              y = edge.content.tf_value.transform.translation.x
              z = edge.content.tf_value.transform.translation.x

              rpy = euler_from_quaternion([
                edge.content.tf_value.transform.rotation.x,
                edge.content.tf_value.transform.rotation.y,
                edge.content.tf_value.transform.rotation.z,
                edge.content.tf_value.transform.rotation.w])

              label = '[pose] (' + str(x) + ', ' + str(x) + ', ' + str(z) + ') [' + (
                str(rpy[0]) + ', ' + str(rpy[1]) + ', ' + str(rpy[2]) + ']')

            if edge.content.type == Content.VPOSE:
              label = '[pose[]] of ' + str(len(edge.content.pose_vector))
            if edge.content.type == Content.VSTRING:
              label = '[tf[]] ' + str(len(edge.content.tf_vector))
            if edge.content.type == Content.ERROR:
              label = '[error] '

            dotcode_factory.add_edge_to_graph(
                dotgraph,
                edge.source_node_id,
                edge.target_node_id,
                label= label,
                url='%s %s %s '.format(edge.source_node_id, edge.target_node_id, label),
                penwidth=1,
                color=[0, 0, 0])
            #if edge.type == "tf":
            #    try:
            #        trans = ros2_knowledge_graphinst.tfBuffer.lookup_transform(
            #        source_frame=edge.target,
            #        target_frame=edge.source,
            #        time=rclpy.time.Time())
            #    except (tf2_ros.LookupException):
            #        continue
#
            #    x = trans._transform._translation.x
            #    y = trans._transform._translation.y
            #    z = trans._transform._translation.z
#
            #    yaw = math.atan2(y, x)
            #    pitch = math.atan2(x, x)
            #    tf_label = '{0:.2f} {1:.2f} {2:.2f} ({3:.2f} {4:.2f})'.format(x, y, z, yaw, pitch)
            #    dotcode_factory.add_edge_to_graph(
            #        dotgraph,
            #        edge.source,
            #        edge.target,
            #        label=tf_label,
            #        url='%s %s %s'.format(edge.source, edge.target, "tf"),
            #        penwidth=1,
            #        color=[0, 0, 255])
#
        # for edge in ros2_knowledge_graphinst.get_graph().edges:
        #     if edge.type == Edge.EDGE_TYPE_STRING:
        #         dotcode_factory.add_edge_to_graph(
        #             dotgraph,
        #             edge.source,
        #             edge.target,
        #             label=edge.string_data,
        #             url='%s %s %s '.format(edge.source, edge.target, edge.string_data),
        #             penwidth=1,
        #             color=[0, 0, 0])
        #     elif edge.type == Edge.EDGE_TYPE_DOUBLE:
        #         dotcode_factory.add_edge_to_graph(
        #             dotgraph,
        #             edge.source,
        #             edge.target,
        #             label=edge.double_data,
        #             url='%s %s %ld '.format(edge.source, edge.target, edge.double_data),
        #             penwidth=1,
        #             color=[0, 0, 0])
        #     elif edge.type == Edge.EDGE_TYPE_TF:
        #         try:
        #             (pose,orientation_q) = self.listener_.lookupTransform(edge.source, edge.target, rospy.Time(0))
        #         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #             continue

        #         # (roll, pitch, yaw) = euler_from_quaternion (orientation_q)
        #         yaw = math.atan2(pose[1], pose[0])
        #         pitch = math.atan2(pose[0], pose[2])
        #         tf_label = '{0:.2f} {1:.2f} {2:.2f} ({3:.2f} {4:.2f})'.format(pose[0], pose[1], pose[2], yaw, pitch)
        #         dotcode_factory.add_edge_to_graph(
        #             dotgraph,
        #             edge.source,
        #             edge.target,
        #             label=tf_label,
        #             url='%s %s %s'.format(edge.source, edge.target, "tf"),
        #             penwidth=1,
        #             color=[0, 0, 255])
        #     else:
        #         print("Error: Edge type unknown")

        return dotgraph

    def generate_dotcode(
        self,
        ros2_knowledge_graphinst,
        dotcode_factory,
        orientation='LR',
        rank='same',  # None, same, min, max, source, sink
        ranksep=0.2,  # vertical distance between layers
        rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
        simplify=False,  # remove double edges
        quiet=False,
        unreachable=False):
        """
        @param ros2_knowledge_graphinst: Ros2KnowledgeGraph instance
        @param ns_filter: nodename filter
        @type  ns_filter: string
        @param topic_filter: topicname filter
        @type  ns_filter: string
        @param graph_mode str: NODE_NODE_GRAPH | NODE_TOPIC_GRAPH | NODE_TOPIC_ALL_GRAPH
        @type  graph_mode: str
        @param orientation: rankdir value (see ORIENTATIONS dict)
        @type  dotcode_factory: object
        @param dotcode_factory: abstract factory manipulating dot language objects
        @param hide_single_connection_topics: if true remove topics with just one connection
        @param hide_dead_end_topics: if true remove topics with publishers only
        @param cluster_namespaces_level: if > 0 places box around members of same namespace
               (TODO: multiple namespace layers)
        @param accumulate_actions: if true each 5 action topic graph nodes are shown as single graph node
        @return: dotcode generated from graph singleton
        @rtype: str
        """
        dotgraph = self.generate_dotgraph(
            ros2_knowledge_graphinst=ros2_knowledge_graphinst,
            dotcode_factory=dotcode_factory,
            orientation=orientation,
            rank=rank,
            ranksep=ranksep,
            rankdir=rankdir,
            simplify=simplify,
            quiet=quiet,
            unreachable=unreachable)
        dotcode = dotcode_factory.create_dot(dotgraph)
        return dotcode
