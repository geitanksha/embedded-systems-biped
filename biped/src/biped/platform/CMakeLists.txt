# Add libraries
add_library(
	biped_platform
	camera.cpp
	compass.cpp
	display.cpp
	encoder.cpp
	imu.cpp
	io_expander.cpp
	neopixel.cpp
	serial.cpp
	time_of_flight.cpp
)

# Set library link targets
target_link_libraries(
	biped_platform PRIVATE
	biped_common
	biped_sensor
	biped_utility
)

# Set arduino library link targets
target_link_arduino_libraries(
	biped_platform PRIVATE
	core
	Wire
	"Adafruit BusIO"
	"Adafruit MPU6050"
	"Adafruit GFX Library"
	"Adafruit NeoPixel"
	"Adafruit SH110X"
	"Adafruit Unified Sensor"
	"DFRobot_BMX160"
	"MCP23018"
	"Kalman Filter Library"
	"STM32duino VL53L4CX"
)
