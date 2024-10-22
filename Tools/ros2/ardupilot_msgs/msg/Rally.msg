geographic_msgs/GeoPoint point
float32 break_altitude # when autolanding, break out of loiter at this alt relative to Home (meters)
float32 land_dir # when the time comes to auto-land, try to land in this direction (degrees)

# seek favorable winds when choosing a landing poi
bool flag_favorable_winds

# do auto land after arriving
bool flag_do_auto_land

# Altitude reference frame
uint8 ALTITUDE_FRAME_ABSOLUTE = 0
uint8 ALTITUDE_FRAME_HOME = 1
uint8 ALTITUDE_FRAME_ORIGIN = 2
uint8 ALTITUDE_FRAME_TERRAIN = 3
uint8 altitude_frame