uint8 mode
uint8 off=1
uint8 velocity=2
uint8 position=3

#Orientation uses the whole thing, depth uses just the z value
#Set to NaN to use current pose as target
geometry_msgs/Quaternion target

bool yawEnabled
