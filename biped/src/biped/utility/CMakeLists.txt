# Add libraries
add_library(
	biped_utility
	math.cpp
)

# Set arduino library link targets
target_link_arduino_libraries(
	biped_utility PRIVATE
	core
)
