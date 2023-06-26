# Add libraries
add_library(
	biped_planner
	maneuver_planner.cpp
	waypoint_planner.cpp
)

# Set library link targets
target_link_libraries(
	biped_planner PRIVATE
	biped_common
	biped_controller
	biped_platform
	biped_sensor
	biped_utility
)
