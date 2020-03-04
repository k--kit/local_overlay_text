Header header

uint32 id
uint8 action

geometry_msgs/Point position
uint8 anchor # anchor point where position is placed
int32 offset_h # horizontal offset from anchor [px] (left to right on display is positive)
int32 offset_v # vertical offset from anchor [px] (top to down on display is positive)

std_msgs/ColorRGBA bg_color

int32 line_width # [px]
float32 text_size # [px]
string font
std_msgs/ColorRGBA fg_color

string text

uint8 ADD = 0
uint8 DELETE = 1

uint8 ANCHOR_TOP = 1
uint8 ANCHOR_VCENTER = 2
uint8 ANCHOR_BOTTOM = 4
uint8 ANCHOR_LEFT = 8
uint8 ANCHOR_HCENTER = 16
uint8 ANCHOR_RIGHT = 32
uint8 ANCHOR_CENTER = 18
