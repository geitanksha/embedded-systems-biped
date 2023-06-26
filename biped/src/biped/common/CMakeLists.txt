# Add libraries
add_library(
	biped_common
	global.cpp
)

# Set library link targets
target_link_libraries(
	biped_common PRIVATE
	biped_actuator
	biped_controller
	biped_planner
	biped_platform
	biped_sensor
	biped_utility
)
