geographic_msgs/GeoPoint point
float32 break_altitude # when autolanding, break out of loiter at this alt (meters)
float32 land_dir # when the time comes to auto-land, try to land in this direction (degrees)

# uint8_t flags; 
# struct {
#     uint8_t favorable_winds : 1; // bit 0 = seek favorable winds when choosing a landing poi
#     uint8_t do_auto_land    : 1; // bit 1 = do auto land after arriving
#     uint8_t alt_frame_valid : 1; // bit 2 = true if following alt frame value should be used, else Location::AltFrame::ABOVE_HOME
#     uint8_t alt_frame       : 2; // Altitude frame following Location::AltFrame enum
#     uint8_t unused          : 3;
# };
uint8 flags
