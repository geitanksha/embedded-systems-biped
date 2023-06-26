# Add executables
add_executable(
	biped
	biped.cpp
)

# Set library link targets
target_link_libraries(
	biped PRIVATE
	biped_common
	biped_platform
	biped_task
)

# Set arduino library link targets
target_link_arduino_libraries(
	biped PRIVATE
	core
	EEPROM
	SPI
	Wire
	"ESP32TimerInterrupt"
)

# Set arduino upload targets
target_enable_arduino_upload(biped)
