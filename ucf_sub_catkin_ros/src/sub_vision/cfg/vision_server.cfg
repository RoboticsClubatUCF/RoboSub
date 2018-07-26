#!/usr/bin/env python
PACKAGE = "sub_vision"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("lowH", int_t, 0, "Lower H Threshold",0,0,255)
gen.add("lowS", int_t, 0, "Lower S Threshold",0,0,255)
gen.add("lowL", int_t, 0, "Lower L Threshold",0,0,255)
gen.add("upH", int_t, 0, "Upper H Threshold",0,0,255)
gen.add("upS", int_t, 0, "Upper S Threshold",0,0,255)
gen.add("upL", int_t, 0, "Upper L Threshold",0,0,255)

exit(gen.generate(PACKAGE, "vision_server", "Thresholds"))