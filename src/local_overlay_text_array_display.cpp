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

#include "local_overlay_text_array_display.h"
#include <OGRE/OgreMaterialManager.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/geometry.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreQuaternion.h>
#include <QFontDatabase>
#include <QPainter>
#include <QStaticText>
#include <QTextDocument>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <geometry_msgs/Pose.h> 
#include <stdexcept>

namespace local_overlay_text
{
  LocalOverlayTextArrayDisplay::LocalOverlayTextArrayDisplay() : Display()
  { 
    update_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
      ros::message_traits::datatype<local_overlay_text::TextArray>(),
      "local_overlay_text::TextArray topic to subscribe to.",
      this, SLOT( updateTopic() ));
    overtake_position_properties_property_ = new rviz::BoolProperty(
      "Overtake Position Properties", false,
      "overtake position properties specified by message such as left, top and font",
      this, SLOT(updateOvertakePositionProperties()));
    overtake_color_properties_property_ = new rviz::BoolProperty(
      "Overtake Color Properties", false,
      "overtake color properties specified by message such as foreground/background color and alpha",
      this, SLOT(updateOvertakeColorProperties()));
    text_size_property_ = new rviz::IntProperty(
      "text size", 12,
      "text size [px]",
      this, SLOT(updateTextSize()));
    text_size_property_->setMin(0);
    line_width_px_property_ = new rviz::IntProperty(
      "line width", 2,
      "line width [px]",
      this, SLOT(updateLineWidth()));
    line_width_px_property_->setMin(0);
    fg_color_property_ = new rviz::ColorProperty(
      "Foreground Color", QColor(25, 255, 240),
      "Foreground Color",
      this, SLOT(updateFGColor()));
    fg_alpha_property_ = new rviz::FloatProperty(
      "Foreground Alpha", 0.8, "Foreground Alpha",
      this, SLOT(updateFGAlpha()));
    fg_alpha_property_->setMin(0.0);
    fg_alpha_property_->setMax(1.0);
    bg_color_property_ = new rviz::ColorProperty(
      "Background Color", QColor(0, 0, 0),
      "Background Color",
      this, SLOT(updateBGColor()));
    bg_alpha_property_ = new rviz::FloatProperty(
      "Background Alpha", 0.8, "Background Alpha",
      this, SLOT(updateBGAlpha()));
    bg_alpha_property_->setMin(0.0);
    bg_alpha_property_->setMax(1.0);

    QFontDatabase database;
    font_families_ = database.families();
    font_property_ = new rviz::EnumProperty(
      "font", "DejaVu Sans Mono",
      "font", this,
      SLOT(updateFont()));
    for (size_t i = 0; i < font_families_.size(); i++) {
      font_property_->addOption(font_families_[i], (int)i);
    }
  }
  
  LocalOverlayTextArrayDisplay::~LocalOverlayTextArrayDisplay()
  {
    onDisable();
    //delete overlay_;
    delete update_topic_property_;
    delete overtake_color_properties_property_;
    delete overtake_position_properties_property_;
    delete text_size_property_;
    delete line_width_px_property_;
    delete bg_color_property_;
    delete bg_alpha_property_;
    delete fg_color_property_;
    delete fg_alpha_property_;
    delete font_property_;
  }

  void LocalOverlayTextArrayDisplay::onEnable()
  {
    for(auto& overlay_kv_ : overlay_map_) {
      auto& overlay_ = overlay_kv_.second;
      if (overlay_) {
        overlay_->show();
      }
    }
    subscribe();
  }

  void LocalOverlayTextArrayDisplay::onDisable()
  {
    for(auto& overlay_kv_ : overlay_map_) {
      auto& overlay_ = overlay_kv_.second;
      if (overlay_) {
        overlay_->hide();
      }
    }
    unsubscribe();
  }
  
  void LocalOverlayTextArrayDisplay::unsubscribe()
  {
    sub_.shutdown();
  }

  void LocalOverlayTextArrayDisplay::subscribe()
  {
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
      sub_ = ros::NodeHandle().subscribe(topic_name, 1, &LocalOverlayTextArrayDisplay::processMessage, this);
    }
  }
  
  void LocalOverlayTextArrayDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }
    
  // only the first time
  void LocalOverlayTextArrayDisplay::onInitialize()
  {
    onEnable();
    updateTopic();
    updateOvertakePositionProperties();
    updateOvertakeColorProperties();
    updateTextSize();
    updateFGColor();
    updateFGAlpha();
    updateBGColor();
    updateBGAlpha();
    updateFont();
    updateLineWidth();
  }
  
  void LocalOverlayTextArrayDisplay::update(float wall_dt, float ros_dt)
  {
    if (!isEnabled()) {
      return;
    }
    for(auto& overlay_kv_ : overlay_map_) {
      auto& overlay_ = overlay_kv_.second;
      if (overlay_) {
        overlay_->update(true); // forcibly update objects to follow viewport change 
      }
    }
  }
  
  void LocalOverlayTextArrayDisplay::processMessage
  (const local_overlay_text::TextArray::ConstPtr& msg)
  {
    if (!isEnabled()) {
      return;
    }
    for (const auto& text : msg->texts) {
      // search id
      auto overlay_kv_ = overlay_map_.find(text.id);
      //ROS_WARN("proces id:%d", text.id);

      if (overlay_kv_ != overlay_map_.end()) {
        // if found and msg is delete, erase object
        auto& overlay_ = overlay_kv_->second;
        if (text.action == local_overlay_text::Text::DELETE) {
          overlay_map_.erase(overlay_kv_);
          continue; // skip following process
        }
      } else {
        // if not found, add new overlay object
        auto insert_ret = overlay_map_.insert(std::make_pair(text.id, 
            LocalOverlayTextObject::Ptr(new LocalOverlayTextObject(context_))));
        if (!insert_ret.second) {
          throw std::runtime_error("object should not be found, but found...");
        }
        overlay_kv_ = insert_ret.first;
      }
      auto& overlay_ = overlay_kv_->second;
      // here overlay_kv_ is surely exist.
      overlay_->setText(text.text);
      overlay_->setPosition(text.header.frame_id, text.header.stamp, text.position);
      overlay_->setOffsetH(text.offset_h);
      overlay_->setOffsetV(text.offset_v);
      overlay_->setAnchor(text.anchor);
      if (!overtake_position_properties_) {
        overlay_->setTextSize(text.text_size);
      }
      if (!overtake_color_properties_) {
        overlay_->setBackGroundColor(text.bg_color);
        overlay_->setFontColor(text.fg_color);
        overlay_->setFont(text.font);
        overlay_->setLineWidth(text.line_width);
      }
      overlay_->show();
    }
  }

  void LocalOverlayTextArrayDisplay::updateOvertakePositionProperties()
  {
    if (!overtake_position_properties_ &&
        overtake_position_properties_property_->getBool()) {
      updateTextSize();
    }
    overtake_position_properties_
      = overtake_position_properties_property_->getBool();
    if (overtake_position_properties_) {
      text_size_property_->show();
    } else {
      text_size_property_->hide();
    }
  }
  
  void LocalOverlayTextArrayDisplay::updateOvertakeColorProperties()
  {
    if (!overtake_color_properties_ &&
        overtake_color_properties_property_->getBool()) {
      // read all the parameters from properties
      updateFGColor();
      updateFGAlpha();
      updateBGColor();
      updateBGAlpha();
      updateFont();
      updateLineWidth();
    }
    overtake_color_properties_ = overtake_color_properties_property_->getBool();
    if (overtake_color_properties_) {
      fg_color_property_->show();
      fg_alpha_property_->show();
      bg_color_property_->show();
      bg_alpha_property_->show();
      line_width_px_property_->show();
      font_property_->show();
    }
    else {
      fg_color_property_->hide();
      fg_alpha_property_->hide();
      bg_color_property_->hide();
      bg_alpha_property_->hide();
      line_width_px_property_->hide();
      font_property_->hide();
    }
  }
  
  void LocalOverlayTextArrayDisplay::updateTextSize()
  { 
    for(auto& overlay_kv_ : overlay_map_) {
      auto& overlay_ = overlay_kv_.second;
      if (overlay_) {
        overlay_->setTextSize(text_size_property_->getInt());
      }
    }
  }

  void LocalOverlayTextArrayDisplay::updateBGColor()
  {
    QColor c = bg_color_property_->getColor();
    std_msgs::ColorRGBA bg_color;
    bg_color.r = c.red() / 255.0;
    bg_color.g = c.green() / 255.0;
    bg_color.b = c.blue() / 255.0;
    // bg_color.a = overlay_->getBackGroundColor().alpha() / 255.0;
    
    for(auto& overlay_kv_ : overlay_map_) {
      auto& overlay_ = overlay_kv_.second;
      if (overlay_) {
        bg_color.a = overlay_->getBackGroundColor().alpha() / 255.0;
        overlay_->setBackGroundColor(bg_color);
      }
    }
  }

  void LocalOverlayTextArrayDisplay::updateBGAlpha()
  {
    for(auto& overlay_kv_ : overlay_map_) {
      auto& overlay_ = overlay_kv_.second;
      if (overlay_) {
        QColor c = overlay_->getBackGroundColor();
        std_msgs::ColorRGBA bg_color;
        bg_color.r = c.red() / 255.0;
        bg_color.g = c.green() / 255.0;
        bg_color.b = c.blue() / 255.0;
        bg_color.a = bg_alpha_property_->getFloat();
        overlay_->setBackGroundColor(bg_color);
      }
    }
  }

  void LocalOverlayTextArrayDisplay::updateFGColor()
  {
    QColor c = fg_color_property_->getColor();
    std_msgs::ColorRGBA fg_color;
    fg_color.r = c.red() / 255.0;
    fg_color.g = c.green() / 255.0;
    fg_color.b = c.blue() / 255.0;
    // fg_color.a = overlay_->getFontColor().alpha() / 255.0;
    
    for(auto& overlay_kv_ : overlay_map_) {
      auto& overlay_ = overlay_kv_.second;
      if (overlay_) {
        fg_color.a = overlay_->getFontColor().alpha() / 255.0;
        overlay_->setFontColor(fg_color);
      }
    }
  }

  void LocalOverlayTextArrayDisplay::updateFGAlpha()
  {
    for(auto& overlay_kv_ : overlay_map_) {
      auto& overlay_ = overlay_kv_.second;
      if (overlay_) {
        QColor c = overlay_->getFontColor();
        std_msgs::ColorRGBA fg_color;
        fg_color.r = c.red() / 255.0;
        fg_color.g = c.green() / 255.0;
        fg_color.b = c.blue() / 255.0;
        fg_color.a = fg_alpha_property_->getFloat();
        overlay_->setFontColor(fg_color);
      }
    }
  }

  void LocalOverlayTextArrayDisplay::updateFont()
  {
    int font_index = font_property_->getOptionInt();
    if (font_index < font_families_.size()) {
      for(auto& overlay_kv_ : overlay_map_) {
        auto& overlay_ = overlay_kv_.second;
        if (overlay_) {
          overlay_->setFont(font_families_[font_index].toStdString());
        }
      }
    } else {
      ROS_FATAL("Unexpected error at selecting font index %d.", font_index);
      return;
    }
  }

  void LocalOverlayTextArrayDisplay::updateLineWidth()
  {
    for(auto& overlay_kv_ : overlay_map_) {
      auto& overlay_ = overlay_kv_.second;
      if (overlay_) {
        overlay_->setLineWidth(line_width_px_property_->getInt());
      }
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( local_overlay_text::LocalOverlayTextArrayDisplay, rviz::Display )
