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

#ifndef RVIZ_SELECT_TOOL_H
#define RVIZ_SELECT_TOOL_H

#include <vector>
#include <stdint.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <rviz_common/tool.hpp>

#include <rviz_cloud_annotation/msg/rectangle_selection_viewport.hpp>

namespace Ogre
{
  class Viewport;
  class Rectangle2D;
  class SceneNode;
  class ManualObject;
}

namespace rviz_common
{
  class ViewportMouseEvent;
}

namespace rviz_rendering
{
  class RenderWindow;
}

namespace rviz_default_plugins
{
  namespace tools
  {
    class InteractionTool;
    class MoveTool;
  }
}

namespace rviz_cloud_annotation
{
  class AnnotationSelectionTool: public rviz_common::Tool
  {
    public:
    typedef int32_t int32;
    typedef uint32_t uint32;
    typedef std::vector<float> FloatVector;

    struct Coords2D
    {
      int32 x;
      int32 y;
      Coords2D(const int32 px,const int32 py): x(px), y(py) {}
    };
    typedef std::vector<Coords2D> Coords2DVector;

    AnnotationSelectionTool();
    virtual ~AnnotationSelectionTool();

    virtual void onInitialize() override;

    virtual void activate() override;
    virtual void deactivate() override;

    virtual int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
    virtual int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;

    virtual void update(float wall_dt,float ros_dt) override;

    private:

    void SendViewportPolylineData(rviz_rendering::RenderWindow * const render_window,
                                  const Coords2DVector & polyline);

    void SendViewportData(const int32 start_x, const int32 start_y,
                          const int32 end_x, const int32 end_y,
                          rviz_rendering::RenderWindow * const render_window);

    void onSetEditMode(const std_msgs::msg::UInt32 &mode);
    void onSetToolType(const std_msgs::msg::UInt32 & type);

    void UpdateCursor();

    void StartRectangleSelection(const int32 start_x, const int32 start_y, rviz_rendering::RenderWindow * const render_window);
    void InitRectangleSelection();

    void StartPolylineSelection(const int32 start_x, const int32 start_y, rviz_rendering::RenderWindow * const render_window);
    void InitPolylineSelection();
    void UpdatePolylineSelection();
    void InitPubSub();

    void ClearSelection();

    rviz_default_plugins::tools::MoveTool * m_move_tool;
    rviz_default_plugins::tools::InteractionTool * m_interaction_tool;

    uint32 m_edit_mode;
    bool m_edit_mode_selectable;
    uint32 m_tool_type;

    rclcpp::Node m_nh;
    rclcpp::Publisher<rviz_cloud_annotation::msg::RectangleSelectionViewport>::SharedPtr m_annotation_selection_pub;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_on_set_mode_sub;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr m_on_set_tool_type_sub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_toggle_none_pub;

    bool m_selecting;
    int32 m_sel_start_x;
    int32 m_sel_start_y;
    int32 m_sel_end_x;
    int32 m_sel_end_y;

    bool m_polyline_selecting;
    bool m_polyline_self_intersect;
    Coords2DVector m_sel_polyline;

    bool m_active;

    Ogre::Rectangle2D * m_selection_rectangle;
    Ogre::ManualObject * m_selection_polyline;
    Ogre::SceneNode * m_selection_rectangle_node;
    Ogre::SceneNode * m_selection_polyline_node;
    rviz_rendering::RenderWindow * m_selection_window;
  };

}

#endif // RVIZ_SELECT_TOOL_H
