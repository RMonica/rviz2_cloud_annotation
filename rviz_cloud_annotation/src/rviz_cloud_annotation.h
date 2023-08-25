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

#include "rviz_cloud_annotation_points/rviz_cloud_annotation_definitions.h"

#ifndef RVIZ_CLOUD_ANNOTATION_H
#define RVIZ_CLOUD_ANNOTATION_H

#define PARAM_NAME_UPDATE_TOPIC                "update_topic"
#define PARAM_DEFAULT_UPDATE_TOPIC             "/rviz_cloud_annotation/update_topic"

#define PARAM_NAME_CLOUD_FILENAME             "cloud_filename"
#define PARAM_DEFAULT_CLOUD_FILENAME          "cloud.pcd"

#define PARAM_NAME_NORMAL_SOURCE              "normal_source"
#define PARAM_VALUE_NORMAL_SOURCE_CLOUD       "cloud"        // from PARAM_NAME_CLOUD_FILENAME itself
#define PARAM_VALUE_NORMAL_SOURCE_OTHER_CLOUD "other_cloud:" // example: "other_cloud:normal_cloud.pcd"
#define PARAM_DEFAULT_NORMAL_SOURCE           PARAM_VALUE_NORMAL_SOURCE_CLOUD

// will attempt to load this at startup
#define PARAM_NAME_ANN_FILENAME_IN            "annotation_read_filename"
#define PARAM_DEFAULT_ANN_FILENAME_IN         "annotation.annotation"

// will save to this when asked by the GUI
#define PARAM_NAME_ANN_FILENAME_OUT           "annotation_write_filename"
#define PARAM_DEFAULT_ANN_FILENAME_OUT        "annotation.annotation"

// will also save this cloud (XYZRGBL)
#define PARAM_NAME_ANNOTATED_CLOUD            "annotation_cloud"
#define PARAM_DEFAULT_ANNOTATED_CLOUD         "annotation.pcd"

#define PARAM_NAME_LABEL_NAMES_FILENAME       "label_names_filename"
#define PARAM_DEFAULT_LABEL_NAMES_FILENAME    "names.txt"

#define PARAM_NAME_AUTOSAVE_TIME              "autosave_time"
#define PARAM_DEFAULT_AUTOSAVE_TIME           (0.0)   // seconds, < 1.0 to disable

#define PARAM_NAME_AUTOSAVE_TIMESTAMP         "autosave_append_timestamp"
#define PARAM_DEFAULT_AUTOSAVE_TIMESTAMP      (false)

#define PARAM_NAME_FRAME_ID                   "frame_id"
#define PARAM_DEFAULT_FRAME_ID                "base_link"

#define PARAM_NAME_POINT_SIZE                 "point_size"
#define PARAM_DEFAULT_POINT_SIZE              (0.005)

// size of the current label for each point
#define PARAM_NAME_LABEL_SIZE                 "label_size"
#define PARAM_DEFAULT_LABEL_SIZE              (0.0025)

// size of the current label for control points
#define PARAM_NAME_CONTROL_LABEL_SIZE         "control_label_size"
#define PARAM_DEFAULT_CONTROL_LABEL_SIZE      (0.02)

// shows labels in the back of the points as well
#define PARAM_NAME_SHOW_POINTS_BACK_LABELS    "show_labels_back"
#define PARAM_DEFAULT_SHOW_POINTS_BACK_LABELS (true)

#define PARAM_NAME_CONTROL_POINT_VISUAL       "control_point_visual"
#define PARAM_VALUE_CONTROL_POINT_VISUAL_LINE "line"
#define PARAM_VALUE_CONTROL_POINT_VISUAL_SPHERE "sphere"
#define PARAM_VALUE_CONTROL_POINT_VISUAL_THREE_SPHERES "three_spheres"
#define PARAM_DEFAULT_CONTROL_POINT_VISUAL    PARAM_VALUE_CONTROL_POINT_VISUAL_SPHERE

// [0..1] 0: size not affected by weight - 1: whole size affected
#define PARAM_NAME_CP_WEIGHT_SCALE_FRACTION   "control_point_weight_scale_fraction"
#define PARAM_DEFAULT_CP_WEIGHT_SCALE_FRACTION (0.5)

#define PARAM_NAME_ZERO_WEIGHT_CP_SHOW        "show_zero_weight_control_points"
#define PARAM_DEFAULT_ZERO_WEIGHT_CP_SHOW     (true)

// from interface to backend
#define PARAM_NAME_SET_EDIT_MODE_TOPIC        "rviz_cloud_annotation/set_edit_mode_topic"
#define PARAM_DEFAULT_SET_EDIT_MODE_TOPIC     "/rviz_cloud_annotation/set_edit_mode"

// from backend to interface
#define PARAM_NAME_SET_EDIT_MODE_TOPIC2       "rviz_cloud_annotation/set_edit_mode_topic2"
#define PARAM_DEFAULT_SET_EDIT_MODE_TOPIC2    "/rviz_cloud_annotation/set_edit_mode2"

#define PARAM_NAME_COLORS_COLS_PER_PAGE       "rviz_cloud_annotation/color_columns_per_page"
#define PARAM_DEFAULT_COLOR_COLS_PER_PAGE     (10)

#define PARAM_NAME_COLORS_ROWS_PER_PAGE       "rviz_cloud_annotation/color_rows_per_page"
#define PARAM_DEFAULT_COLOR_ROWS_PER_PAGE     (2)

#define PARAM_NAME_POINT_SIZE_CHANGE_MULT     "rviz_cloud_annotation/point_change_size_multiplier"
#define PARAM_DEFAULT_POINT_SIZE_CHANGE_MULT  (0.2)

#define PARAM_NAME_SET_CURRENT_LABEL_TOPIC    "rviz_cloud_annotation/set_current_label_topic"
#define PARAM_DEFAULT_SET_CURRENT_LABEL_TOPIC "/rviz_cloud_annotation/set_current_label"

#define PARAM_NAME_CURRENT_LABEL_TOPIC        "rviz_cloud_annotation/set_current_label_topic2"
#define PARAM_DEFAULT_CURRENT_LABEL_TOPIC     "/rviz_cloud_annotation/set_current_label2"

#define PARAM_NAME_SAVE_TOPIC                 "rviz_cloud_annotation/save_topic"
#define PARAM_DEFAULT_SAVE_TOPIC              "/rviz_cloud_annotation/save"

#define PARAM_NAME_RESTORE_TOPIC              "rviz_cloud_annotation/restore_topic"
#define PARAM_DEFAULT_RESTORE_TOPIC           "/rviz_cloud_annotation/restore"

#define PARAM_NAME_CLEAR_TOPIC                "rviz_cloud_annotation/clear_topic"
#define PARAM_DEFAULT_CLEAR_TOPIC             "/rviz_cloud_annotation/clear"

// from RViz to backend
#define PARAM_NAME_SET_NAME_TOPIC             "rviz_cloud_annotation/set_name_topic"
#define PARAM_DEFAULT_SET_NAME_TOPIC          "/rviz_cloud_annotation/set_name"

// from backend to RViz
#define PARAM_NAME_SET_NAME_TOPIC2            "rviz_cloud_annotation/set_name_topic2"
#define PARAM_DEFAULT_SET_NAME_TOPIC2         "/rviz_cloud_annotation/set_name2"

// from backend to RViz
#define PARAM_NAME_POINT_COUNT_UPDATE_TOPIC   "rviz_cloud_annotation/point_count_update"
#define PARAM_DEFAULT_POINT_COUNT_UPDATE_TOPIC "/rviz_cloud_annotation/point_count_update"

#define PARAM_NAME_VIEW_LABEL_TOPIC           "rviz_cloud_annotation/view_labels_topic"
#define PARAM_DEFAULT_VIEW_LABEL_TOPIC        "/rviz_cloud_annotation/view_labels"

#define PARAM_NAME_VIEW_CONTROL_POINTS_TOPIC  "rviz_cloud_annotation/view_control_points_topic"
#define PARAM_DEFAULT_VIEW_CONTROL_POINTS_TOPIC "/rviz_cloud_annotation/view_control_points"

#define PARAM_NAME_VIEW_CLOUD_TOPIC           "rviz_cloud_annotation/view_cloud_topic"
#define PARAM_DEFAULT_VIEW_CLOUD_TOPIC        "/rviz_cloud_annotation/view_cloud"

#define PARAM_NAME_UNDO_TOPIC                 "rviz_cloud_annotation/undo_topic"
#define PARAM_DEFAULT_UNDO_TOPIC              "/rviz_cloud_annotation/undo"

#define PARAM_NAME_REDO_TOPIC                 "rviz_cloud_annotation/redo_topic"
#define PARAM_DEFAULT_REDO_TOPIC              "/rviz_cloud_annotation/redo"

#define PARAM_NAME_UNDO_REDO_STATE_TOPIC      "rviz_cloud_annotation/undo_redo_state_topic"
#define PARAM_DEFAULT_UNDO_REDO_STATE_TOPIC   "/rviz_cloud_annotation/undo_redo_state"

#define PARAM_NAME_POINT_SIZE_CHANGE_TOPIC    "rviz_cloud_annotation/point_size_change_topic"
#define PARAM_DEFAULT_POINT_SIZE_CHANGE_TOPIC "/rviz_cloud_annotation/point_size_change"

#define PARAM_NAME_CONTROL_POINT_WEIGHT_TOPIC "rviz_cloud_annotation/point_weight_topic"
#define PARAM_DEFAULT_CONTROL_POINT_WEIGHT_TOPIC "/rviz_cloud_annotation/point_weight"

#define PARAM_NAME_CONTROL_POINT_MAX_WEIGHT_TOPIC "rviz_cloud_annotation/point_max_weight_topic"
#define PARAM_DEFAULT_CONTROL_POINT_MAX_WEIGHT_TOPIC "/rviz_cloud_annotation/point_max_weight"

#define PARAM_NAME_GOTO_FIRST_UNUSED_TOPIC    "rviz_cloud_annotation/goto_first_unused_topic"
#define PARAM_DEFAULT_GOTO_FIRST_UNUSED_TOPIC "/rviz_cloud_annotation/goto_first_unused"

#define PARAM_NAME_GOTO_LAST_UNUSED_TOPIC     "rviz_cloud_annotation/goto_last_unused_topic"
#define PARAM_DEFAULT_GOTO_LAST_UNUSED_TOPIC  "/rviz_cloud_annotation/goto_last_unused"

#define PARAM_NAME_GOTO_NEXT_UNUSED_TOPIC     "rviz_cloud_annotation/goto_next_unused_topic"
#define PARAM_DEFAULT_GOTO_NEXT_UNUSED_TOPIC  "/rviz_cloud_annotation/goto_next_unused"

#define PARAM_NAME_GOTO_FIRST_TOPIC           "rviz_cloud_annotation/goto_first_topic"
#define PARAM_DEFAULT_GOTO_FIRST_TOPIC        "/rviz_cloud_annotation/goto_first"

#define PARAM_NAME_RECT_SELECTION_TOPIC       "rviz_cloud_annotation/rect_selection_topic"
#define PARAM_DEFAULT_RECT_SELECTION_TOPIC    "/rviz_cloud_annotation/rect_selection"

#define PARAM_NAME_TOOL_TYPE_TOPIC            "rviz_cloud_annotation/tool_type_topic"
#define PARAM_DEFAULT_TOOL_TYPE_TOPIC         "/rviz_cloud_annotation/tool_type"

#define PARAM_NAME_TOGGLE_NONE_TOPIC       "rviz_cloud_annotation/toggle_none_topic"
#define PARAM_DEFAULT_TOGGLE_NONE_TOPIC    "/rviz_cloud_annotation/toggle_none"

// parameters for smart labeling
  // neighborhood graph distance
#define PARAM_NAME_NEIGH_SEARCH_DISTANCE      "neighborhood_search_distance" // DEPRECATED
#define PARAM_DEFAULT_NEIGH_SEARCH_DISTANCE   (0.02)

#define EDIT_MODE_NONE                        (0)
#define EDIT_MODE_CONTROL_POINT               (1)
#define EDIT_MODE_ERASER                      (2)
#define EDIT_MODE_COLOR_PICKER                (3)
#define EDIT_MODE_MAX                         (4)

#define TOOL_TYPE_SINGLE_PICK                 (0)
#define TOOL_TYPE_DEEP_SQUARE                 (1)
#define TOOL_TYPE_SHALLOW_SQUARE              (2)
#define TOOL_TYPE_SHALLOW_POLY                (3)
#define TOOL_TYPE_MAX                         (4)

#define POINT_SIZE_CHANGE_BIGGER              (1)
#define POINT_SIZE_CHANGE_SMALLER             (-1)
#define POINT_SIZE_CHANGE_RESET               (0)

#endif // RVIZ_CLOUD_ANNOTATION_H
