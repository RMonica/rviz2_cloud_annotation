/*
 * Copyright (c) 2016-2017, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture
 *   University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef RVIZ_CLOUD_ANNOTATION_CLASS_H
#define RVIZ_CLOUD_ANNOTATION_CLASS_H

#include "rviz_cloud_annotation.h"
#include "rviz_cloud_annotation_points/rviz_cloud_annotation_points.h"
#include "rviz_cloud_annotation_points/point_neighborhood.h"
#include "rviz_cloud_annotation_points/rviz_cloud_annotation_undo.h"
#include <rviz_cloud_annotation/msg/undo_redo_state.hpp>
#include <rviz_cloud_annotation/msg/rectangle_selection_viewport.hpp>

// STL
#include <stdint.h>
#include <cmath>
#include <string>
#include <fstream>
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/colors.h>

class RVizCloudAnnotation
{
  public:
  typedef visualization_msgs::msg::InteractiveMarker InteractiveMarker;
  typedef visualization_msgs::msg::Marker Marker;
  typedef interactive_markers::InteractiveMarkerServer InteractiveMarkerServer;
  typedef std::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
  typedef visualization_msgs::msg::InteractiveMarkerFeedback InteractiveMarkerFeedback;
  typedef std::shared_ptr<visualization_msgs::msg::InteractiveMarkerFeedback> InteractiveMarkerFeedbackPtr;
  typedef std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> InteractiveMarkerFeedbackConstPtr;
  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;
  typedef pcl::Normal PointNormal;
  typedef pcl::PointCloud<PointNormal> PointNormalCloud;
  typedef pcl::PointXYZRGB PointXYZRGB;
  typedef pcl::PointCloud<PointXYZRGB> PointXYZRGBCloud;
  typedef pcl::PointXYZRGBL PointXYZRGBL;
  typedef pcl::PointCloud<PointXYZRGBL> PointXYZRGBLCloud;
  typedef pcl::KdTreeFLANN<PointXYZRGBNormal> KdTree;
  typedef RVizCloudAnnotationPoints::CPData ControlPointData;
  typedef RVizCloudAnnotationPoints::CPDataVector ControlPointDataVector;

  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef uint8_t uint8;
  typedef unsigned int uint;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::vector<float> FloatVector;
  typedef std::vector<bool> BoolVector;

  struct Int32PolyTriangle
  {
    int32 x[3];
    int32 y[3];
    Int32PolyTriangle(int32 x1,int32 y1,int32 x2,int32 y2,int32 x3,int32 y3)
    {
      x[0] = x1; x[1] = x2; x[2] = x3;
      y[0] = y1; y[1] = y2; y[2] = y3;
    }

    // -1: inside 1: outside 0: on border
    int32 Contains(const int32 x,const int32 y) const;
  };
  typedef std::vector<Int32PolyTriangle> Int32PolyTriangleVector;

  enum ControlPointVisual
  {
    CONTROL_POINT_VISUAL_SPHERE,
    CONTROL_POINT_VISUAL_THREE_SPHERES,
    CONTROL_POINT_VISUAL_LINE,
  };

  RVizCloudAnnotation(rclcpp::Node &nh);

  void LoadCloud(const std::string & filename,const std::string & normal_source,PointXYZRGBNormalCloud & cloud);

  void onSave(const std_msgs::msg::String & filename_msg) { Save(); }
  void onAutosave() { Save(true); }
  void Save(const bool is_autosave = false);

  std::string AppendTimestampBeforeExtension(const std::string & filename);

  void onRestore(const std_msgs::msg::String & filename_msg)
  {
    std::string filename = filename_msg.data.empty() ? m_annotation_filename_out : filename_msg.data;
    Restore(filename);
  }

  void Restore(const std::string & filename);

  void onClear(const std_msgs::msg::UInt32 & label_msg);

  void onClickOnCloud(const InteractiveMarkerFeedbackConstPtr & feedback_ptr);
  std::string GetClickType(const std::string & marker_name,uint64 & label_out) const;
  uint64 GetClickedPointId(const InteractiveMarkerFeedback & click_feedback,bool & ok);

  void onRectangleSelectionViewport(const rviz_cloud_annotation::msg::RectangleSelectionViewport & msg);
  void VectorSelection(const Uint64Vector & ids);
  Uint64Vector RectangleSelectionToIds(const Eigen::Matrix4f proj_matrix,
                                       const Eigen::Affine3f camera_pose_inv,
                                       const PointXYZRGBNormalCloud & cloud,
                                       const uint32 start_x,
                                       const uint32 start_y,
                                       const uint32 width,
                                       const uint32 height,
                                       const Int32PolyTriangleVector tri_cond,
                                       const float point_size,
                                       const float focal_length,
                                       const bool is_deep_selection);

  void SetCurrentLabel(const uint64 label);

  void SetEditMode(const uint64 new_edit_mode);

  void onViewLabels(const std_msgs::msg::Bool & msg)
  {
    m_view_labels = msg.data;

    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),true);
  }

  void onViewControlPoints(const std_msgs::msg::Bool & msg)
  {
    m_view_control_points = msg.data;

    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),true);
  }

  void onViewCloud(const std_msgs::msg::Bool & msg)
  {
    m_view_cloud = msg.data;

    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),true);
  }

  void onSetCurrentLabel(const std_msgs::msg::UInt32 & msg)
  {
    SetCurrentLabel(msg.data);
  }

  void onSetEditMode(const std_msgs::msg::UInt32 & msg)
  {
    SetEditMode(msg.data);
  }

  void onToggleNoneMode(const std_msgs::msg::Empty & msg)
  {
    if ((m_edit_mode == EDIT_MODE_NONE) && (m_prev_edit_mode != EDIT_MODE_NONE))
      SetEditMode(m_prev_edit_mode);
    else if (m_edit_mode != EDIT_MODE_NONE)
      SetEditMode(EDIT_MODE_NONE);
  }

  void onSetName(const std_msgs::msg::String & msg)
  {
    m_undo_redo.SetNameForLabel(m_current_label,msg.data);
    RCLCPP_INFO(m_log, "rviz_cloud_annotation: label %u is now named \"%s\".",uint(m_current_label),msg.data.c_str());
    SendName();
    SendUndoRedoState();
  }

  void onUndo(const std_msgs::msg::Empty &);
  void onRedo(const std_msgs::msg::Empty &);

  void onPointSizeChange(const std_msgs::msg::Int32 & msg);

  void onControlPointWeightChange(const std_msgs::msg::UInt32 & msg);

  void onGotoFirstUnused(const std_msgs::msg::Empty &);
  void onGotoLastUnused(const std_msgs::msg::Empty &);
  void onGotoFirst(const std_msgs::msg::Empty &);
  void onGotoNextUnused(const std_msgs::msg::Empty &);

  void SendName();

  void SendUndoRedoState();

  void SendPointCounts(const Uint64Vector & labels);

  Uint64Vector RangeUint64(const uint64 start,const uint64 end) const
  {
    Uint64Vector result(end - start);
    for (uint64 i = start; i < end; i++)
      result[i - start] = i;
    return result;
  }

  float NANToZero(const float a)
  {
    return std::isfinite(a) ? a : 0.0;
  }

  void SendCloudMarker(const bool apply);

  void ClearControlPointsMarker(const Uint64Vector & indices,const bool apply);

  void SendControlPointsMarker(const Uint64Vector & changed_labels,const bool apply);

  void SendControlPointMaxWeight();

  InteractiveMarker ControlPointsToMarker(const PointXYZRGBNormalCloud & cloud,
                                          const ControlPointDataVector & control_points,
                                          const uint64 label, const bool interactive);
  InteractiveMarker LabelsToMarker(const PointXYZRGBNormalCloud & cloud,
                                   const Uint64Vector & labels,
                                   const uint64 label,const bool interactive);

  InteractiveMarker CloudToMarker(const PointXYZRGBNormalCloud & cloud,const bool interactive);

  private:
  rclcpp::Node & m_nh;
  rclcpp::Logger m_log;
  InteractiveMarkerServerPtr m_interactive_marker_server;
  PointXYZRGBNormalCloud::Ptr m_cloud;

  RVizCloudAnnotationPoints::ConstPtr m_annotation;
  RVizCloudAnnotationUndo m_undo_redo;

  KdTree::Ptr m_kdtree;

  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_set_edit_mode_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_toggle_none_sub;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_set_current_label_sub;

  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_set_edit_mode_pub;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_set_current_label_pub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_set_name_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_set_name_pub;

  rclcpp::Subscription<rviz_cloud_annotation::msg::RectangleSelectionViewport>::SharedPtr m_rect_selection_sub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_save_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_restore_sub;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_clear_sub;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_undo_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_redo_sub;
  rclcpp::Publisher<rviz_cloud_annotation::msg::UndoRedoState>::SharedPtr m_undo_redo_state_pub;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_point_size_change_sub;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_goto_first_unused_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_goto_last_unused_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_goto_first_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_goto_next_unused_sub;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_view_control_points_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_view_cloud_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_view_labels_sub;
  bool m_view_cloud;
  bool m_view_labels;
  bool m_view_control_points;

  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr m_point_count_update_pub;

  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_control_points_weight_sub;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_control_point_weight_max_weight_pub;
  uint32 m_control_point_weight_step;
  uint32 m_control_point_max_weight;

  rclcpp::TimerBase::SharedPtr m_autosave_timer;
  bool m_autosave_append_timestamp;

  std::string m_frame_id;
  float m_point_size;
  float m_label_size;
  float m_control_label_size;

  float m_point_size_multiplier;
  float m_point_size_change_multiplier;

  bool m_show_points_back_labels;
  float m_cp_weight_scale_fraction;
  ControlPointVisual m_control_points_visual;
  bool m_show_zero_weight_control_points;

  uint64 m_current_label;
  uint64 m_edit_mode;
  uint64 m_prev_edit_mode;

  PointNeighborhood::ConstPtr m_point_neighborhood;

  std::string m_annotation_filename_in;
  std::string m_annotation_filename_out;
  std::string m_ann_cloud_filename_out;
  std::string m_label_names_filename_out;
};

#endif // RVIZ_CLOUD_ANNOTATION_CLASS_H
