# Add libraries
add_library(
	biped_controller
	controller.cpp
	open_loop_controller.cpp
	pid_controller.cpp
)

# Set library link targets
target_link_libraries(
	biped_controller PRIVATE
	biped_common
	biped_platform
	biped_sensor
	biped_utility
)
