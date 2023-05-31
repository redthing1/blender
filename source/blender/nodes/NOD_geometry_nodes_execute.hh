/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/**
 * For evaluation, geometry node groups are converted to a lazy-function graph. The generated graph
 * is cached per node group, so it only has to be generated once after a change.
 *
 * Node groups are *not* inlined into the lazy-function graph. This could be added in the future as
 * it might improve performance in some cases, but generally does not seem necessary. Inlining node
 * groups also has disadvantages like making per-node-group caches less useful, resulting in more
 * overhead.
 *
 * Instead, group nodes are just like all other nodes in the lazy-function graph. What makes them
 * special is that they reference the lazy-function graph of the group they reference.
 *
 * During lazy-function graph generation, a mapping between the #bNodeTree and
 * #lazy_function::Graph is build that can be used when evaluating the graph (e.g. for logging).
 */

#include "BLI_compute_context.hh"
#include "BLI_function_ref.hh"
#include "BLI_multi_value_map.hh"

#include "FN_lazy_function_graph.hh"
#include "FN_lazy_function_graph_executor.hh"

#include "BKE_idprop.hh"

struct bNodeTree;
struct bNodeSocket;
struct Depsgraph;
struct GeometrySet;
struct IDProperty;
struct Object;
namespace blender::nodes {
struct GeoNodesLFUserData;
namespace geo_eval_log {
class GeoModifierLog;
}  // namespace geo_eval_log
}  // namespace blender::nodes

namespace blender::nodes {

/**
 * \return Whether using an attribute to input values of this type is supported.
 */
bool socket_type_has_attribute_toggle(const bNodeSocket &socket);

/**
 * \return Whether using an attribute to input values of this type is supported, and the node
 * group's input for this socket accepts a field rather than just single values.
 */
bool input_has_attribute_toggle(const bNodeTree &node_tree, const int socket_index);

bool id_property_type_matches_socket(const bNodeSocket &socket, const IDProperty &property);

std::unique_ptr<IDProperty, bke::idprop::IDPropertyDeleter> id_property_create_from_socket(
    const bNodeSocket &socket);

GeometrySet execute_geometry_nodes(const bNodeTree &btree,
                                   const IDProperty *properties,
                                   const ComputeContext &base_compute_context,
                                   GeometrySet input_geometry,
                                   FunctionRef<void(nodes::GeoNodesLFUserData &)> fill_user_data);

void update_input_properties_from_node_tree(const bNodeTree &tree,
                                            const IDProperty *old_properties,
                                            IDProperty &properties);

void update_output_properties_from_node_tree(const bNodeTree &tree,
                                             const IDProperty *old_properties,
                                             IDProperty &properties);

}  // namespace blender::nodes
