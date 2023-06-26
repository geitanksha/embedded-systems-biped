/**
 *  @file   parameter.h
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Parameter constant expression header.
 *
 *  This file defines the parameter constant expressions.
 */

/*
 *  Include guard.
 */
#ifndef COMMON_PARAMETER_H_
#define COMMON_PARAMETER_H_

/*
 *  External headers.
 */
#include <cstddef>
#include <cstdint>
#include <freertos/FreeRTOS.h>

/*
 *  Project headers.
 */
#include "common/type.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/*
 *  Address parameter namespace.
 */
namespace AddressParameter
{
constexpr uint8_t display = 0x3C;   //!< Display I2C address.
constexpr uint8_t eeprom_serial_number = 0x00;  //!< Serial number EEPROM address.
constexpr uint8_t imu_mpu6050 = 0x69;   //!< MPU6050 IMU I2C address.
constexpr uint8_t io_expander_a = 0x00; //!< I/O expander A I2C address.
constexpr uint8_t io_expander_b = 0x07; //!< I/O expander B I2C address.
constexpr uint8_t time_of_flight_left = 0x40;   //!< Left time-of-flight I2C address.
constexpr uint8_t time_of_flight_middle = 0x41; //!< Middle time-of-flight I2C address.
constexpr uint8_t time_of_flight_right = 0x42;  //!< Right time-of-flight I2C address.
}   // namespace AddressParameter

/*
 *  Camera parameter namespace.
 */
namespace CameraParameter
{
constexpr size_t frame_buffer_count = 2;    //!< Number of frame buffers.
constexpr int jpeg_quality = 12;    //!< JPEG image quality.
constexpr char stream_part_boundary[] = "123456789000000000000987654321"; //!< Stream part boundary string.
constexpr int xclk_frequency = 20000000;    //!< X clock frequency, in Hertz.
}   // namespace CameraParameter

/*
 *  Compass parameter namespace.
 */
namespace CompassParameter
{
/*
 *  The compass are easily affected by any nearby magnetic fields.
 *  Therefore, the 0 degree produced by the compass might misalign
 *  with the true north. Tune the Z attitude offset below such that
 *  the Biped points to the direction of your liking, e.g., true north,
 *  with a 0-degree Z attitude controller reference.
 *
 *  To persist a compass calibration result, print the calibration data
 *  struct in the best-effort task, record the values, and update
 *  the calibration parameters below. The calibration would then persist
 *  across reboots.
 *
 *  Tune the calibration time and calibration controller reference below
 *  such that the Biped would rotate 360 degrees half-way through the
 *  calibration time.
 */
constexpr double attitude_z_offset = 0;   //!< Z attitude offset, in degrees.
constexpr double calibration_offset_x = -5.15625;  //!< X compass calibration offset.
constexpr double calibration_offset_y = -32.0769; //!< Y compass calibration offset.
constexpr double calibration_offset_z = 137.442;    //!< Z compass calibration offset.
constexpr double calibration_scaler_x = 0.481503;   //!< X compass calibration scaler.
constexpr double calibration_scaler_y = 0.550976;   //!< Y compass calibration scaler.
constexpr double calibration_scaler_z = 0.778637;   //!< Z compass calibration scaler.
constexpr double calibration_sign_x = 1;   //!< X compass calibration sign.
constexpr double calibration_sign_y = 1;    //!< Y compass calibration sign.
constexpr double calibration_sign_z = 1;    //!< Z compass calibration sign.
constexpr double calibration_range = 120;   //!< Compass calibration range.
constexpr double calibration_time = 24; //!< Compass calibration time, in seconds.
constexpr double calibration_controller_reference = 50; //!< Compass calibration controller reference.
}   // namespace CompassParameter

/*
 *  Controller parameter namespace.
 */
namespace ControllerParameter
{
/*
 *  For lab 7, tune all the controller gains. For
 *  lab 8, tune all the controller saturations.
 *  See the controller class for more details.
 */
constexpr double attitude_y_active = 20; //!< Maximum Y attitude for controller to remain active, in degrees.
constexpr double attitude_y_gain_proportional = -3500; //!< Y attitude PID controller proportional gain.
constexpr double attitude_y_gain_differential = -37; //!< Y attitude PID controller differential gain.
constexpr double attitude_y_gain_integral = 0;  //!< Y attitude PID controller integral gain.
constexpr double attitude_y_gain_integral_max = 0; //!< Y attitude PID controller maximum integral error.
constexpr double attitude_z_gain_proportional = 35; //!< Z attitude PID controller proportional gain.
constexpr double attitude_z_gain_differential = 35; //!< Z attitude PID controller differential gain.
constexpr double attitude_z_gain_integral = 0;  //!< Z attitude PID controller integral gain.
constexpr double attitude_z_gain_integral_max = 0; //!< Z attitude PID controller maximum integral error.
constexpr double attitude_z_gain_open_loop = -1;    //!< Z attitude open-loop controller gain.
constexpr double position_x_gain_proportional = 1000; //!< X position PID controller proportional gain.
constexpr double position_x_gain_differential = 1200; //!< X position PID controller differential gain.
constexpr double position_x_gain_integral = 0;  //!< X position PID controller integral gain.
constexpr double position_x_gain_integral_max = 0; //!< X position PID controller maximum integral error.
constexpr double position_x_saturation_input_lower = -0.4; //!< X position controller input saturation upper bound.
constexpr double position_x_saturation_input_upper = 0.4; //!< X position controller input saturation lower bound.
}   // namespace ControllerParameter

/*
 *  Display parameter namespace.
 */
namespace DisplayParameter
{
constexpr double acceleration_z_rotation = 0; //!< Display rotation X acceleration threshold, in meters per second squared.
constexpr uint16_t height = 64; //!< Display height, in pixels.
constexpr double low_pass_filter_beta = 0.9;    //!< Low-pass filter beta parameter.
constexpr uint32_t postclk = 1000000;   //!< Post-clock frequency, in Hertz.
constexpr uint32_t preclk = 1000000;    //!< Pre-clock frequency, in Hertz.
constexpr int8_t reset_pin = -1;    //!< Reset pin.
constexpr uint8_t rotation_upright = 1; //!< Upright rotation.
constexpr uint8_t rotation_inverted = 3;    //!< Inverted rotation.
constexpr uint8_t text_size = 1;    //!< Display text size.
constexpr uint16_t width = 128; //!< Display width, in pixels.
}   // namespace DisplayParameter

/*
 *  EEPROM parameter namespace.
 */
namespace EEPROMParameter
{
constexpr size_t size = 4;  //!< EEPROM size, in bytes.
}   // namespace EEPROMParameter

/*
 *  Encoder parameter namespace.
 */
namespace EncoderParameter
{
constexpr double low_pass_filter_beta = 0.7;    //!< Low-pass filter beta parameter.
constexpr unsigned steps_per_meter = 7400;  //!< Number of encoder steps per meter.
}   // namespace EncoderParameter

/*
 *  I/O expander parameter namespace.
 */
namespace IOExpanderParameter
{
constexpr size_t num_port_pins = 8; //!< Number of pins per port.
constexpr size_t num_ports = 2; //!< Number of ports.
}   // namespace IOExpanderParameter

/*
 *  Kalman filter parameter namespace.
 */
namespace KalmanFilterParameter
{
constexpr float q_angle = 0.001;    //!< Q angle parameter.
constexpr float q_bias = 0.005; //!< Q bias parameter.
constexpr float r_measure = 0.5;    //!< R measure parameter.
}   // namespace KalmanFilterParameter

/*
 *  Median filter parameter namespace.
 */
namespace MedianFilterParameter
{
constexpr size_t window_size = 10;  //!< Window size parameter.
}   // namespace MedianFilterParameter

/*
 *  Motor parameter namespace.
 */
namespace MotorParameter
{
constexpr unsigned pwm_min = 0; //!< Minimum pulse-width modulation (PWM) value.
constexpr unsigned pwm_max = 255;   //!< Maximum pulse-width modulation (PWM) value.
}   // namespace MotorParameter

/*
 *  NeoPixel parameter namespace.
 */
namespace NeoPixelParameter
{
constexpr uint8_t brightness_max = 30;  //!< Maximum brightness.
constexpr uint8_t brightness_min = 10;  //!< Minimum brightness
constexpr uint16_t size = 4;    //!< Size of the NeoPixel LED array.
}   // namespace NeoPixelParameter

/*
 *  Network parameter namespace.
 */
namespace NetworkParameter
{
constexpr char passphrase[] = "";   //!< Wi-Fi passphrase.
constexpr char ssid[] = "IllinoisNet_Guest";    //!< Wi-Fi SSID.
}   // namespace NetworkParameter

/*
 *  Period parameter namespace.
 */
namespace PeriodParameter
{
/*
 *  The fast domain period is used by the interrupt timer for
 *  generating hardware timer interrupts for the real-time task.
 *  The slow domain period specifies how often the slow domain
 *  tasks in the real-time task function runs.
 *
 *  Check the real-time task function for more details.
 */
constexpr double fast = 0.005;  //!< Fast domain period, in seconds.
constexpr double slow = 0.04;   //!< Slow domain period, in seconds.
}   // namespace PeriodParameter

/*
 *  Serial parameter namespace.
 */
namespace SerialParameter
{
constexpr unsigned long baud_rate = 115200; //!< Baud rate.
constexpr LogLevel log_level_max = LogLevel::trace; //!< Maximum log level.
}   // namespace SerialParameter

/*
 *  Task parameter namespace.
 */
namespace TaskParameter
{
constexpr int core_0 = 0;   //!< Core 0 number.
constexpr int core_1 = 1;   //!< Core 1 number.
constexpr unsigned int priority_min = 0;    //!< Minimum priority.
constexpr unsigned int priority_max = configMAX_PRIORITIES; //!< Maximum priority.
constexpr uint32_t stack_size = 4096;   //!< Task stack size, in bytes.
}   // namespace TaskParameter
}   // namespace biped

#endif  // COMMON_PARAMETER_H_
