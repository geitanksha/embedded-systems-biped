# Add libraries
add_library(
	biped_actuator
	actuator.cpp
)

# Set library link targets
target_link_libraries(
	biped_actuator PRIVATE
	biped_common
	biped_controller
	biped_utility
)

# Set arduino library link targets
target_link_arduino_libraries(
	biped_actuator PRIVATE
	core
)
