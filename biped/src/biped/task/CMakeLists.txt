# Add libraries
add_library(
	biped_task
	interrupt.cpp
	task.cpp
)

# Set library link targets
target_link_libraries(
	biped_task PRIVATE
	biped_common
	biped_platform
)

# Set arduino library link targets
target_link_arduino_libraries(
	biped_task PRIVATE
	core
	WiFi
)
