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

#ifndef RVIZ_CLOUD_ANNOTATION_PLUGIN_H
#define RVIZ_CLOUD_ANNOTATION_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>

// STL
#include <stdint.h>
#include <vector>

#include <rviz_cloud_annotation/msg/undo_redo_state.hpp>

class QLabel;
class QPushButton;
class QButtonGroup;
class QLineEdit;
class QAction;
class QToolButton;
class QSlider;
class QMenu;

namespace pcl { class RGB; }

namespace rviz_cloud_annotation
{

  class QRVizCloudAnnotation: public rviz_common::Panel
  {
    Q_OBJECT;

    typedef uint64_t uint64;
    typedef int64_t int64;
    typedef uint32_t uint32;
    typedef std::vector<uint64> Uint64Vector;
    typedef std::vector<QPushButton *> PQPushButtonVector;

    public:
    QRVizCloudAnnotation(QWidget* parent = NULL);
    virtual ~QRVizCloudAnnotation();

    virtual void onInitialize() override;

    private Q_SLOTS:
    void onSetEditMode(int mode);
    void onSetToolType(int type);

    void onLabelButtonSelected(int id);
    void onPlusLabel();
    void onMinusLabel();
    void onPageUp();
    void onPageDown();

    void onSave();
    void onRestore();
    void onClear();
    void onClearCurrent();

    void onUndo();
    void onRedo();

    void onSendName();

    void onViewCloudToggled(const bool checked);
    void onViewControlPointsToggled(const bool checked);
    void onViewLabelsToggled(const bool checked);

    void onGotoFirstUnused();
    void onGotoLastUnused();
    void onGotoFirst();
    void onGotoNextUnused();

    void onSmallerPoints();
    void onBiggerPoints();
    void onResetPointsSize();

    void onControlPointWeightSliderMoved(int new_value);
    void onControlPointWeightSliderSet(int new_value);
    void onControlPointWeightInc();
    void onControlPointWeightDec();
    void onControlPointWeightMax();
    void onControlPointWeightMin();
    void onSetControlPointMaxWeight(const std_msgs::msg::UInt32 & msg);

    private:
    void SetCurrentEditMode(const uint64 mode);

    void FillColorPageButtons();
    void FillPointCounts();
    void FillColorPageButtonStylesheet();

    void SetCurrentLabel(const uint64 label,const uint64 page);

    void onSetCurrentLabel(const std_msgs::msg::UInt32 & label);
    void onSetEditMode2(const std_msgs::msg::UInt32 &mode);
    void onPointCountUpdate(const std_msgs::msg::UInt64MultiArray & counters);
    void onUndoRedoState(const rviz_cloud_annotation::msg::UndoRedoState & msg);

    void onSetName(const std_msgs::msg::String &name);

    uint64 GetPageForLabel(const uint64 label) const;
    uint64 GetLabelFromPageAndId(const uint64 page,const int id) const;
    uint64 GetFirstLabelForPage(const uint64 page) const;
    uint64 GetLastLabelForPage(const uint64 page) const;

    void SetUndoText(const std::string & text);
    void SetRedoText(const std::string & text);

    static void ColorToHex(const pcl::RGB & color,char hex[7]);

    uint64 m_current_edit_mode;

    // 0 for the eraser
    uint64 m_current_label;
    uint64 m_current_page;

    rclcpp::Node m_nh;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_set_edit_mode_pub;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_set_current_label_pub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_save_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_restore_pub;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_clear_pub;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_goto_first_unused_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_goto_last_unused_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_goto_first_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_goto_next_unused_pub;

    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_set_edit_mode_sub;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_set_current_label_sub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_set_name_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_set_name_sub;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_view_cloud_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_view_labels_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_view_control_points_pub;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_redo_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_undo_pub;
    rclcpp::Subscription<rviz_cloud_annotation::msg::UndoRedoState>::SharedPtr m_undo_redo_state_sub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_point_size_change_pub;

    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_control_points_weight_pub;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_control_point_max_weight_sub;

    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_tool_type_pub;

    QPushButton * m_edit_none_button;
    QPushButton * m_edit_control_point_button;
    QPushButton * m_edit_eraser_button;
    QPushButton * m_edit_color_picker_button;
    QButtonGroup * m_toolbar_group;

    QPushButton * m_tool_single_button;
    QPushButton * m_tool_shallow_square_button;
    QPushButton * m_tool_deep_square_button;
    QPushButton * m_tool_shallow_poly_button;
    QButtonGroup * m_tooltype_group;

    QAction * m_prev_page_action;
    QAction * m_next_page_action;
    QAction * m_next_label_action;
    QAction * m_prev_label_action;

    QAction * m_prev_weight_action;
    QAction * m_next_weight_action;
    QAction * m_min_weight_action;
    QAction * m_max_weight_action;
    QMenu * m_weight_menu;

    QAction * m_undo_action;
    QAction * m_redo_action;

    QLabel * m_current_page_label;

    QLabel * m_current_control_point_weight_label;
    QSlider * m_current_control_point_weight_slider;
    uint32 m_control_point_weight_max;

    PQPushButtonVector m_page_buttons;
    QButtonGroup * m_page_button_group;

    Uint64Vector m_point_counters;
    rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr m_point_count_update_sub;

    QLineEdit * m_set_name_edit;

    uint64 m_color_cols_per_page;
    uint64 m_color_rows_per_page;
  };

}

#endif // RVIZ_CLOUD_ANNOTATION_PLUGIN_H
