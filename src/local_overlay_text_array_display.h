// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, k--kit.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the k--kit. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef _RVIZ_PLUGIN_LOCAL_OVERLAY_TEXT_ARRAY_DISPLAY_H_
#define _RVIZ_PLUGIN_LOCAL_OVERLAY_TEXT_ARRAY_DISPLAY_H_

#include "local_overlay_text/TextArray.h"
#ifndef Q_MOC_RUN
#include <rviz/display.h>
#include "overlay_utils.h"
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>
#include <std_msgs/ColorRGBA.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <stdexcept>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreQuaternion.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/geometry.h>
#include <QFontDatabase>
#include <QPainter>
#include <QStaticText>
#include <QTextDocument>
#endif

namespace local_overlay_text
{
  class LocalOverlayTextObject
  {
    public:
#if ROS_VERSION_MINIMUM(1,12,0)
    typedef std::shared_ptr<LocalOverlayTextObject> Ptr;
#else
    typedef boost::shared_ptr<LocalOverlayTextObject> Ptr;
#endif
    static const int ANCHOR_TOP = 1, 
      ANCHOR_VCENTER = 2, 
      ANCHOR_BOTTOM = 4, 
      ANCHOR_LEFT = 8, 
      ANCHOR_HCENTER = 16, 
      ANCHOR_RIGHT = 32, 
      ANCHOR_CENTER = 18;

    LocalOverlayTextObject(rviz::DisplayContext* context) {
      if (!context) {
        throw std::runtime_error("context is null");
      }
      context_ = context;
      
      static int count = 0;
      rviz::UniformStringStream ss;
      ss << "LocalOverlayTextDisplayObject" << count++;
      overlay_.reset(new OverlayObject(ss.str()));
      overlay_->hide();
    }
    ~LocalOverlayTextObject() {
      if (overlay_) {
        overlay_->hide();
      }
    }
    protected:
    rviz::DisplayContext* context_;
    OverlayObject::Ptr overlay_;
    
    std::string frame_id_;
    ros::Time time_;
    geometry_msgs::Point position_;

    uint texture_width_px_ = 1;
    uint texture_height_px_ = 1;
    QColor bg_color_;
    QColor fg_color_;
    uint text_size_= 0;
    uint line_width_ = 0;
    std::string text_;
    std::string font_;
    int left_px_ = 0;
    int top_px_ = 0;
    int offset_h_ = 0;
    int offset_v_ = 0;
    int anchor_ = 0;

    bool require_update_texture_ = false;

    bool localToRenderArea(
      const std::string& frame_id, const ros::Time& time, const geometry_msgs::Point& local_pos, 
      int& x_px, int& y_px) {
      // local coords -> world coords
      Ogre::Vector3 world_pos;
      Ogre::Quaternion world_orient;
      geometry_msgs::Pose pose;
      pose.position = local_pos;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      if (!context_->getFrameManager()->transform(
        frame_id, time, pose, world_pos, world_orient)) 
      {
        std::string error;
        context_->getFrameManager()->transformHasProblems(
          frame_id, time, error);
        return false;
      }
      // world coords -> screen coords [px]
      Ogre::Vector2 screen_point 
        = rviz::project3DPointToViewportXY(
          context_->getViewManager()->getCurrent()->getCamera()->getViewport(), world_pos);
      x_px = screen_point.x;
      y_px = screen_point.y;
      // const Ogre::Camera* m_c = context_->getViewManager()->getCurrent()->getCamera();
      // Ogre::Matrix4 tx_world2clip = m_c->getProjectionMatrixRS() * m_c->getViewMatrix();
      // Ogre::Vector3 clipped_point = tx_world2clip * world_pos;
      // // https://forums.ogre3d.org/viewtopic.php?t=2519
      // const Ogre::Viewport* m_vp = m_c->getViewport();
      // int height = m_vp->getActualHeight(); // render area height [px]
      // int width = m_vp->getActualWidth(); // render area width [px]
      // int nCWidth = (width/2); 
      // int nCHeight = (height/2);
      // x_px = nCWidth * (1.0 + clipped_point.x);
      // y_px = nCHeight * (1.0 - clipped_point.y);
      return true;
    }

    public:
    void setText(const std::string& text) {
      text_ = text;
      require_update_texture_ = true;
    }
    void setFont(const std::string& font) {
      font_ = font;
      require_update_texture_ = true;
    }
    void setTextSize(const uint& text_size) {
      text_size_ = text_size;
      require_update_texture_ = true;
    }
    void setLineWidth(const int& line_width) {
      line_width_ = line_width;
      require_update_texture_ = true;
    }
    void setOffsetH(const int& offset_h) {
      offset_h_ = offset_h;
      require_update_texture_ = true;
    }
    void setOffsetV(const int& offset_v) {
      offset_v_ = offset_v;
      require_update_texture_ = true;
    }
    void setBackGroundColor (const std_msgs::ColorRGBA& bg_color) {
      bg_color_ = QColor(bg_color.r * 255.0,
                         bg_color.g * 255.0,
                         bg_color.b * 255.0,
                         bg_color.a * 255.0);
      require_update_texture_ = true;
    }
    const QColor& getBackGroundColor() const {
      return bg_color_;
    }
    void setFontColor (const std_msgs::ColorRGBA& fg_color) {
      fg_color_ = QColor(fg_color.r * 255.0,
                         fg_color.g * 255.0,
                         fg_color.b * 255.0,
                         fg_color.a * 255.0);
      require_update_texture_ = true;
    }
    const QColor& getFontColor() const {
      return fg_color_;
    }

    void setPosition(
      const std::string& frame_id, const ros::Time& time, const geometry_msgs::Point& position) {
      frame_id_ = frame_id;
      time_ = time;
      position_ = position;
      require_update_texture_ = true;
    }

    void setAnchor(const int& anchor) {
      anchor_ = anchor;
      require_update_texture_ = true;
    }
    void show() {
      if (overlay_) {
        overlay_->show();
      }
    }
    void hide() {
      if (overlay_) {
        overlay_->hide();
      }
    }
    void update(bool forced = false) {
      if (!forced && !require_update_texture_) {
        return;
      }
      if (!overlay_) {
        return;
      }
      // calc text size
      {
        overlay_->updateTextureSize(texture_width_px_, texture_height_px_);
        uint16_t w = overlay_->getTextureWidth();
        uint16_t h = overlay_->getTextureHeight();

        ScopedPixelBuffer buffer = overlay_->getBuffer();
        QImage Hud = buffer.getQImage(*overlay_, bg_color_);
        QPainter painter( &Hud );
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(fg_color_, line_width_ || 1, Qt::SolidLine));

        // font
        if (text_size_ != 0) {
          //QFont font = painter.font();
          QFont font(font_.length() > 0 ? font_.c_str(): "Liberation Sans");
          font.setPointSize(text_size_);
          //font.setBold(true);
          painter.setFont(font);
        }
        if (text_.length() > 0) {
          QString txt = QString::fromStdString(text_);
          QFontMetrics fm2(painter.font());
          // calc render area size
          // fm2.boundingRect(txt) does not work somehow
          //  (multiline size is not calculated correctly). may be bug
          double width = 0.0;
          double height = 0.0;
          for (QString line : txt.split("\n")) {
            QRect line_rect = fm2.boundingRect(line);
            width = (line_rect.width() > width) ? line_rect.width() : width;
            height += line_rect.height();
          }
          QRect tmp(0,0,width + text_size_,height);
          painter.drawText(tmp,Qt::AlignLeft | Qt::AlignTop,txt);
          // because of bug in qt, fontmetrics with html statictext is not correct: 
          //    https://forum.qt.io/topic/91642/qstatictext-size-bug-when-using-rich-text-and-word-wrap
          // QRect text_rect2 = fm2.boundingRect(static_text.text().remove(QRegExp("<[^>]*>")));
          // painter.drawStaticText(0,0, static_text);
          w = tmp.width();
          h = tmp.height();
        }
        painter.end();
        texture_width_px_ = w;
        texture_height_px_ = h;
        overlay_->updateTextureSize(w, h);
        overlay_->setDimensions(w, h);
      }

      // --------------------------------
      // calc and update position
      {
        if (!localToRenderArea(frame_id_, time_, position_, left_px_, top_px_)) {
          overlay_->hide();
          return;
        }
        // calc anchor point
        int anchor_offset_v = 0; 
        if (anchor_ & ANCHOR_TOP) {
          anchor_offset_v = 0;
        } else if (anchor_ & ANCHOR_VCENTER) {
          anchor_offset_v = -(int)texture_height_px_ / 2;
        } else if (anchor_ & ANCHOR_BOTTOM) {
          anchor_offset_v = -(int)texture_height_px_;
        } else {
          anchor_offset_v = 0; // default is top
        } 
        int anchor_offset_h = 0; 
        if (anchor_ & ANCHOR_LEFT) {
          anchor_offset_h = 0;
        } else if (anchor_ & ANCHOR_HCENTER) {
          anchor_offset_h = -(int)texture_width_px_ / 2;
        } else if (anchor_ & ANCHOR_RIGHT) {
          anchor_offset_h = -(int)texture_width_px_;
        } else {
          anchor_offset_h = 0; // default is left
        }

        // add offset
        left_px_ += offset_h_ + anchor_offset_h;
        top_px_ += offset_v_ + anchor_offset_v;
        overlay_->setPosition(left_px_, top_px_);
      }
      require_update_texture_ = false;
    }
  };

  class LocalOverlayTextArrayDisplay
  : public rviz::Display
  {
    Q_OBJECT
  public:
    LocalOverlayTextArrayDisplay();
    virtual ~LocalOverlayTextArrayDisplay();
  protected:
    std::map<uint32_t, LocalOverlayTextObject::Ptr> overlay_map_;
    
    bool overtake_color_properties_;
    bool overtake_position_properties_;
    QStringList font_families_;

    ros::Subscriber sub_;
    
    virtual void onInitialize();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void onEnable();
    virtual void onDisable();
    virtual void update(float wall_dt, float ros_dt);

    rviz::RosTopicProperty* update_topic_property_;
    rviz::BoolProperty* overtake_position_properties_property_;
    rviz::BoolProperty* overtake_color_properties_property_;
    rviz::IntProperty* text_size_property_;
    rviz::IntProperty* line_width_px_property_;
    rviz::ColorProperty* bg_color_property_;
    rviz::FloatProperty* bg_alpha_property_;
    rviz::ColorProperty* fg_color_property_;
    rviz::FloatProperty* fg_alpha_property_;
    rviz::EnumProperty* font_property_;
  protected Q_SLOTS:
    void updateTopic();
    void updateOvertakePositionProperties();
    void updateOvertakeColorProperties();
    void updateTextSize();
    void updateFGColor();
    void updateFGAlpha();
    void updateBGColor();
    void updateBGAlpha();
    void updateFont();
    void updateLineWidth();
  protected:
    void processMessage(const local_overlay_text::TextArray::ConstPtr& msg);
  };
}
#endif
