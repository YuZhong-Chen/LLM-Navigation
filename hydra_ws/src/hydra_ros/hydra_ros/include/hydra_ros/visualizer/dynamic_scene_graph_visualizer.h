/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <functional>
#include <string>
#include <vector>

#include "hydra_ros/visualizer/config_manager.h"
#include "hydra_ros/visualizer/dsg_visualizer_plugin.h"
#include "hydra_ros/visualizer/visualizer_types.h"
#include "hydra_ros/visualizer/visualizer_utilities.h"

namespace hydra {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

void clearPrevMarkers(const std_msgs::Header& header,
                      const std::set<NodeId>& curr_nodes,
                      const std::string& ns,
                      std::set<NodeId>& prev_nodes,
                      MarkerArray& msg);

class DynamicSceneGraphVisualizer {
 public:
  explicit DynamicSceneGraphVisualizer(const ros::NodeHandle& nh);

  virtual ~DynamicSceneGraphVisualizer() = default;

  void addPlugin(const std::shared_ptr<DsgVisualizerPlugin>& plugin) {
    plugins_.push_back(plugin);
  }

  void clearPlugins() { plugins_.clear(); }

  void start(bool periodic_redraw = false);

  bool redraw();

  void setGraphUpdated() { need_redraw_ = true; }

  void setGraph(const DynamicSceneGraph::Ptr& scene_graph, bool need_reset = true);

  void reset();

  bool graphIsSet() const { return scene_graph_.operator bool(); }

  void setNeedRedraw() { need_redraw_ = true; }

  DynamicSceneGraph::Ptr getGraph() const { return scene_graph_; }

  void setLayerColorFunction(LayerId layer, const ColorFunction& func);

  void addUpdateCallback(
      const std::function<void(const DynamicSceneGraph::Ptr&)>& func) {
    callbacks_.push_back(func);
  }

 protected:
  virtual void resetImpl(const std_msgs::Header& header, MarkerArray& msg);

  virtual void redrawImpl(const std_msgs::Header& header, MarkerArray& msg);

  virtual void drawLayer(const std_msgs::Header& header,
                         const SceneGraphLayer& layer,
                         const LayerConfig& config,
                         MarkerArray& msg);

  virtual void drawLayerMeshEdges(const std_msgs::Header& header,
                                  LayerId layer_id,
                                  const std::string& ns,
                                  MarkerArray& msg);

 protected:
  void deleteMultiMarker(const std_msgs::Header& header,
                         const std::string& ns,
                         MarkerArray& msg);

  void addMultiMarkerIfValid(const Marker& marker, MarkerArray& msg);

  void displayLoop(const ros::WallTimerEvent&);

  void deleteLayer(const std_msgs::Header& header,
                   const SceneGraphLayer& layer,
                   MarkerArray& msg);

  inline std::string getDynamicNodeNamespace(char layer_prefix) const {
    return dynamic_node_ns_prefix_ + layer_prefix;
  }

  inline std::string getDynamicEdgeNamespace(char layer_prefix) const {
    return dynamic_edge_ns_prefix_ + layer_prefix;
  }

  inline std::string getDynamicLabelNamespace(char layer_prefix) const {
    return dynamic_label_ns_prefix_ + layer_prefix;
  }

  inline std::string getLayerNodeNamespace(LayerId layer) const {
    return node_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerEdgeNamespace(LayerId layer) const {
    return edge_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerLabelNamespace(LayerId layer) const {
    return label_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerBboxNamespace(LayerId layer) const {
    return bbox_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerBboxEdgeNamespace(LayerId layer) const {
    return bbox_ns_prefix_ + "edges_" + std::to_string(layer);
  }
  inline std::string getLayerBoundaryNamespace(LayerId layer) const {
    return boundary_ns_prefix_ + std::to_string(layer);
  }
  inline std::string getLayerBoundaryEllipseNamespace(LayerId layer) const {
    return boundary_ellipse_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerBoundaryEdgeNamespace(LayerId layer) const {
    return boundary_ns_prefix_ + "edges_" + std::to_string(layer);
  }

 private:
  void drawDynamicLayer(const std_msgs::Header& header,
                        const DynamicSceneGraphLayer& layer,
                        const DynamicLayerConfig& config,
                        const VisualizerConfig& viz_config,
                        size_t viz_idx,
                        MarkerArray& msg);

  void deleteLabel(const std_msgs::Header& header, char prefix, MarkerArray& msg);

  void deleteDynamicLayer(const std_msgs::Header& header,
                          char prefix,
                          MarkerArray& msg);

  void drawDynamicLayers(const std_msgs::Header& header, MarkerArray& msg);

  Color getParentColor(const SceneGraphNode& node) const;

 protected:
  ros::NodeHandle nh_;
  ros::WallTimer visualizer_loop_timer_;
  ConfigManager::Ptr config_manager_;

  bool need_redraw_;
  bool periodic_redraw_;
  std::string visualizer_frame_;
  DynamicSceneGraph::Ptr scene_graph_;
  std::map<LayerId, ColorFunction> layer_colors_;
  std::list<std::function<void(const DynamicSceneGraph::Ptr&)>> callbacks_;

  const std::string node_ns_prefix_ = "layer_nodes_";
  const std::string edge_ns_prefix_ = "layer_edges_";
  const std::string label_ns_prefix_ = "layer_labels_";
  const std::string bbox_ns_prefix_ = "layer_bounding_boxes_";
  const std::string boundary_ns_prefix_ = "layer_polygon_boundaries_";
  const std::string boundary_ellipse_ns_prefix_ = "layer_ellipsoid_boundaries_";
  const std::string mesh_edge_ns_ = "mesh_object_connections";
  const std::string interlayer_edge_ns_prefix_ = "interlayer_edges_";
  const LayerId mesh_edge_source_layer_ = DsgLayers::MESH_PLACES;
  const std::string dynamic_node_ns_prefix_ = "dynamic_nodes_";
  const std::string dynamic_edge_ns_prefix_ = "dynamic_edges_";
  const std::string dynamic_label_ns_prefix_ = "dynamic_label_";

  std::set<std::string> published_multimarkers_;
  std::map<LayerId, std::set<NodeId>> prev_labels_;
  std::map<LayerId, std::set<NodeId>> curr_labels_;
  std::set<std::string> published_dynamic_labels_;

  ros::Publisher dsg_pub_;
  ros::Publisher dynamic_layers_viz_pub_;
  std::list<std::shared_ptr<DsgVisualizerPlugin>> plugins_;

  // Scene grapher parser
  void dsg_parser(const DynamicSceneGraph& graph, const std::map<LayerId, LayerConfig>& configs);
};

}  // namespace hydra
