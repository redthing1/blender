/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edcurves
 */

#include "ED_curves.h"
#include "ED_object.h"
#include "ED_screen.h"
#include "ED_select_utils.h"
#include "ED_view3d.h"

#include "WM_api.h"

#include "BKE_attribute_math.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_context.h"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_layer.h"
#include "BKE_lib_id.h"
#include "BKE_node_runtime.hh"
#include "BKE_object.h"
#include "BKE_report.h"

#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"
#include "RNA_prototypes.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "FN_lazy_function_execute.hh"

#include "NOD_geometry_nodes_execute.hh"
#include "NOD_geometry_nodes_lazy_function.hh"

#include "curves_intern.hh"

namespace blender::ed::curves {

static int run_node_group_exec(bContext *C, wmOperator *op)
{
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  Object *object = CTX_data_active_object(C);
  if (!object) {
    return OPERATOR_CANCELLED;
  }
  eObjectMode mode = eObjectMode(object->mode);

  char name[MAX_ID_NAME];
  RNA_string_get(op->ptr, "name", name);
  const ID *id = BKE_libblock_find_name(CTX_data_main(C), ID_NT, name);
  if (!id) {
    return OPERATOR_CANCELLED;
  }
  const bNodeTree &node_tree = reinterpret_cast<const bNodeTree &>(*id);
  const nodes::GeometryNodesLazyFunctionGraphInfo *lf_graph_info =
      nodes::ensure_geometry_nodes_lazy_function_graph(node_tree);
  if (lf_graph_info == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Cannot evaluate node group");
    return OPERATOR_CANCELLED;
  }

  uint objects_len = 0;
  Object **objects = BKE_view_layer_array_from_objects_in_mode_unique_data(
      scene, view_layer, CTX_wm_view3d(C), &objects_len, mode);

  for (Object *object : Span(objects, objects_len)) {
    Curves &curves_id = *static_cast<Curves *>(object->data);
    GeometrySet original_geometry = GeometrySet::create_with_curves(
        &curves_id, GeometryOwnershipType::Editable);

    nodes::GeoNodesOperatorData operator_eval_data{};
    operator_eval_data.depsgraph = depsgraph;
    operator_eval_data.self_object = object;

    bke::ModifierComputeContext compute_context{nullptr, "actually not a modifier"};
    GeometrySet new_geometry = nodes::execute_geometry_nodes(
        node_tree,
        op->properties,
        compute_context,
        original_geometry,
        [&](nodes::GeoNodesLFUserData &user_data) {
          user_data.operator_data = &operator_eval_data;
          user_data.log_socket_values = false;
        });

    if (Curves *new_curves_id = new_geometry.get_curves_for_write()) {
      if (new_curves_id != &curves_id) {
        curves_id.geometry.wrap() = std::move(new_curves_id->geometry.wrap());
      }
    }
    else {
      curves_id.geometry.wrap() = {};
    }

    DEG_id_tag_update(&curves_id.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA, &curves_id);
  }

  MEM_SAFE_FREE(objects);

  return OPERATOR_FINISHED;
}

static int run_node_group_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  char name[MAX_ID_NAME];
  RNA_string_get(op->ptr, "name", name);
  const ID *id = BKE_libblock_find_name(CTX_data_main(C), ID_NT, name);
  if (!id) {
    return OPERATOR_CANCELLED;
  }
  const bNodeTree &node_tree = reinterpret_cast<const bNodeTree &>(*id);

  nodes::update_input_properties_from_node_tree(node_tree, op->properties, *op->properties);
  nodes::update_output_properties_from_node_tree(node_tree, op->properties, *op->properties);

  return run_node_group_exec(C, op);
}

void CURVES_OT_node_group(wmOperatorType *ot)
{
  ot->name = "Run Node Group";
  ot->idname = __func__;
  ot->description = "Execute a node group on curves";  // TODO: Retrieve from node group.

  ot->invoke = run_node_group_invoke;
  ot->exec = run_node_group_exec;
  ot->poll = editable_curves_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  PropertyRNA *prop = RNA_def_string(ot->srna,
                                     "name",
                                     nullptr,
                                     MAX_ID_NAME - 2,
                                     "Name",
                                     "Name of the data-block to use by the operator");
  RNA_def_property_flag(prop, PropertyFlag(PROP_SKIP_SAVE | PROP_HIDDEN));
}

}  // namespace blender::ed::curves
