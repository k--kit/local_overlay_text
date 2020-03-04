#!/usr/bin/env python

import re

from local_overlay_text.msg import Text
from local_overlay_text.msg import TextArray
import rospy

def publish_text(event):
    text_array = TextArray()
    
    text2 = Text()
    text2.id = 1
    text2.header.stamp = rospy.Time.now()
    text2.header.frame_id = 'camera_depth_frame'
    text2.position.x = event.current_real.secs % 10 + event.current_real.nsecs * 1e-9
    text2.position.y = 10.0
    text2.position.z = 0.0
    text2.fg_color.a = 1.0
    text2.fg_color.r = 0.0
    text2.fg_color.g = 1.0
    text2.fg_color.b = 1.0
    text2.bg_color.a = 0.1
    # text2.line_width = 2
    # text2.offset_h = 120
    # text2.offset_v = 100
    text2.anchor = Text.ANCHOR_HCENTER | Text.ANCHOR_BOTTOM
    text2.text_size = 15
    text2.text = 'Lorem ipsum dolor sit amet, consectetur adipiscing elit, \nTimer called at ' + str(event.current_real)
    text_array.texts.append(text2)

    text = Text()
    text.id = 0
    text.header.stamp = rospy.Time.now()
    text.header.frame_id = 'camera_depth_frame'
    text.position.x = 0.0
    text.position.y = (event.current_real.secs % 10) + event.current_real.nsecs * 1e-9
    text.position.z = 0.0
    text.fg_color.a = 1.0
    text.fg_color.r = 0.0
    text.fg_color.g = 1.0
    text.fg_color.b = 0.0
    text.bg_color.a = 0.3
    # text.line_width = 1
    # text.offset_h = 120
    # text.offset_v = 100
    text.anchor = Text.ANCHOR_HCENTER | Text.ANCHOR_BOTTOM
    text.text_size = 12
    text.text = 'Lorem ipsum dolor sit amet, consectetur adipiscing elit, \nTimer called at ' + str(event.current_real)
    text_array.texts.append(text)

    text_array.header.stamp = rospy.Time.now()
    text_array.header.frame_id = 'camera_depth_frame'
    pub.publish(text_array)

if __name__ == "__main__":
    rospy.init_node("overlay_text_sample")
    pub = rospy.Publisher("~output", TextArray, queue_size=1)
    rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
