/**
 *  @file   global.h
 *  @author Simon Yu
 *  @date   12/01/2022
 *  @brief  Global variable header.
 *
 *  This file defines the global variables.
 */

/*
 *  Include guard.
 */
#ifndef COMMON_GLOBAL_H_
#define COMMON_GLOBAL_H_

/*
 *  External headers.
 */
#include <freertos/FreeRTOS.h>
#include <memory>
#include <mutex>
#include <freertos/task.h>

/*
 *  Forward declaration.
 */
class ESP32TimerInterrupt;

/*
 *  Biped namespace.
 */
namespace biped
{
/*
 *  Forward declaration.
 */
class Actuator;
class Camera;
class Controller;
class IOExpander;
class NeoPixel;
class Planner;
class Sensor;

extern std::shared_ptr<Actuator> actuator_; //!< Actuator object shared pointer.
extern std::shared_ptr<Camera> camera_; //!< Camera object shared pointer.
extern std::shared_ptr<Controller> controller_; //!< Controller object shared pointer.
extern std::shared_ptr<IOExpander> io_expander_a_;  //!< I/O expander A object shared pointer.
extern std::shared_ptr<IOExpander> io_expander_b_;  //!< I/O expander B object shared pointer.
extern std::shared_ptr<NeoPixel> neopixel_; //!< NeoPixel object shared pointer.
extern std::shared_ptr<Planner> planner_;   //!< Planner object shared pointer.
extern std::shared_ptr<Sensor> sensor_; //!< Sensor object shared pointer.
extern std::shared_ptr<ESP32TimerInterrupt> timer_; //!< ESP32 timer interrupt object shared pointer.

extern std::mutex mutex_wire_;  //!< I2C driver object mutex.
extern std::unique_lock<std::mutex> lock_wire_; //!< I2C driver object mutex lock.

extern TaskHandle_t task_handle_camera_;    //!< Camera task handle.
extern TaskHandle_t task_handle_io_expander_a_interrupt_; //!< I/O expander A interrupt task handle.
extern TaskHandle_t task_handle_io_expander_b_interrupt_; //!< I/O expander B interrupt task handle.
extern TaskHandle_t task_handle_real_time_; //!< Real-time task handle.
extern TaskHandle_t task_handle_wifi_;  //!< Wi-Fi task handle.

/*
 *  The domain timer below is used for determining whether a time
 *  length of slow domain period has passed since the last domain
 *  timer reset in the real-time task function. The slow domain
 *  period is a multiple of fast domain period.
 */
extern unsigned long execution_time_real_time_task_; //!< Real-time task execution time, in microseconds.
extern unsigned long interval_real_time_task_;  //!< Real-time task interval, in microseconds.
extern double timer_domain_;    //!< Period domain timer, in seconds.

extern unsigned serial_number_; //!< Serial number.
}   // namespace biped

#endif  // COMMON_GLOBAL_H_
