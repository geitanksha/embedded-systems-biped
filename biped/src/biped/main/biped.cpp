/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Biped main program source.
 *
 *  This file implements the Biped main program.
 */

 /*
  *  External headers.
  */
#include <EEPROM.h>
#include <esp_intr_alloc.h>
#include <ESP32TimerInterrupt.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

  /*
   *  Project headers.
   */
#include "actuator/actuator.h"
#include "platform/camera.h"
#include "controller/controller.h"
#include "platform/display.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "task/interrupt.h"
#include "planner/maneuver_planner.h"
#include "common/pin.h"
#include "sensor/sensor.h"
#include "platform/serial.h"
#include "task/task.h"
#include "planner/waypoint_planner.h"

   /*
    *  Use biped namespace.
    */
using namespace biped;

/**
 *  @brief  Main program setup function.
 *
 *  This function creates, configures, and launched drivers, objects,
 *  and tasks. The function also sets pin modes and attaches interrupt
 *  handlers to their corresponding pins.
 */
void
setup()
{

    /*
     *  Set pin mode for the I/O expander interrupt pins using
     *  the Arduino pin mode function. Use pull-up if the pin
     *  mode is input.
     *  See the parameter header for details.
     */
    pinMode(ESP32Pin::io_expander_a_interrupt, INPUT_PULLUP);
    pinMode(ESP32Pin::io_expander_b_interrupt, INPUT_PULLUP);

    /*
     *  Set I2C driver object (Wire) SDA and SCL pins and set the
     *  serial object maximum log level.
     *  See the parameter header for details.
     */
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    Serial::setLogLevelMax(SerialParameter::log_level_max);

    /*
     *  Initialize I2C driver (Wire), EEPROM driver (EEPROM),
     *  display, and serial objects.
     *  See the parameter header for details.
     */
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    Serial::initialize();
    Display::initialize();
    /*
     *  Instantiate all objects and store their shared pointers.
     *
     *  Note that the order of instantiation matters! The camera
     *  object has to be instantiated first, then the I/O expanders,
     *  and then the rest of the objects.
     *
     *  See the global and parameter header for details.
     */
    camera_ = std::make_shared<Camera>();
    io_expander_a_ = std::make_shared<IOExpander>(AddressParameter::io_expander_a);
    io_expander_b_ = std::make_shared<IOExpander>(AddressParameter::io_expander_b);
    actuator_ = std::make_shared<Actuator>();
    controller_ = std::make_shared<Controller>();
    neopixel_ = std::make_shared<NeoPixel>();
    planner_ = std::make_shared<ManeuverPlanner>();
    // planner_ = std::make_shared<ManeuverPlanner>();
    // std::shared_ptr<Planner> planner_; //!< Planner object shared pointer.
    sensor_ = std::make_shared<Sensor>();
    timer_ = std::make_shared<ESP32TimerInterrupt>(2);

    /*
     *  Read and store the serial number from the EEPROM.
     *  See the global and parameter header for details.
     */
    serial_number_ = static_cast<unsigned int>(EEPROM.read(AddressParameter::eeprom_serial_number));

    /*
     *  Set controller periods.
     *  See the controller class for details.
     *
     *  Remember to set both the fast and slow domain
     *  periods, if applicable.
     */
    controller_->setPeriod(PeriodParameter::fast, true);
    controller_->setPeriod(PeriodParameter::slow, false);

    /*
     *  Create I/O expander interrupt tasks using the
     *  FreeRTOS xTaskCreatePinnedToCore function. Set
     *  the task descriptive names to be their task
     *  function names. The tasks have the highest
     *  priority. Pin both tasks to core 1.
     *  See the global and parameter header for details.
     */
    xTaskCreatePinnedToCore(ioExpanderAInterruptTask, "ioExpanderAInterruptTask", TaskParameter::stack_size, NULL,
        TaskParameter::priority_max, &task_handle_io_expander_a_interrupt_, 1);
    xTaskCreatePinnedToCore(ioExpanderBInterruptTask, "ioExpanderBInterruptTask", TaskParameter::stack_size, NULL,
        TaskParameter::priority_max, &task_handle_io_expander_b_interrupt_, 1);


    /*
     *  Attach the I/O expander and encoder interrupt handlers.
     *  Attach the I/O expander interrupt handlers first in rising mode.
     *  Then, attach the encoder interrupt handlers in change mode.
     *
     *  See the interrupt and parameter header for details.
     */
    biped::attachInterrupt(ESP32Pin::io_expander_a_interrupt, ioExpanderAInterruptHandler, RISING);
    biped::attachInterrupt(ESP32Pin::io_expander_b_interrupt, ioExpanderBInterruptHandler, RISING);
    biped::attachInterrupt(ESP32Pin::motor_left_encoder_a, encoderLeftAInterruptHandler, CHANGE);
    biped::attachInterrupt(ESP32Pin::motor_left_encoder_b, encoderLeftBInterruptHandler, CHANGE);
    biped::attachInterrupt(ESP32Pin::motor_right_encoder_a, encoderRightAInterruptHandler, CHANGE);
    biped::attachInterrupt(ESP32Pin::motor_right_encoder_b, encoderRightBInterruptHandler, CHANGE);

    /*
     *  Set pin mode for the push button pins using
     *  the I/O expander pin mode functions. Use
     *  pull-up if the pin mode is input.
     *  See the parameter header for details.
     */
    io_expander_a_->pinModePortA(IOExpanderAPortAPin::pushbutton_a, INPUT_PULLUP);
    io_expander_a_->pinModePortA(IOExpanderAPortAPin::pushbutton_b, INPUT_PULLUP);
    io_expander_a_->pinModePortB(IOExpanderAPortBPin::pushbutton_c, INPUT_PULLUP);

    /*
     *  Attach the push button interrupt handlers using
     *  the I/O expander functions in falling mode.
     *  See the interrupt and parameter header for details.
     */
    io_expander_a_->attachInterruptPortA(IOExpanderAPortAPin::pushbutton_a, pushButtonAInterruptHandler, FALLING);
    io_expander_a_->attachInterruptPortA(IOExpanderAPortAPin::pushbutton_b, pushButtonBInterruptHandler, FALLING);

    /*
     *  Create real-time, Wi-Fi, and camera tasks using the
     *  FreeRTOS xTaskCreatePinnedToCore function, in that
     *  order. Set the task descriptive names to be their task
     *  function names. The real-time task has the second
     *  highest priority and the other two tasks have the
     *  lowest priority. Pin all tasks to core 1.
     *  See the global and parameter header for details.
     */
    xTaskCreatePinnedToCore(realTimeTask, "realTimeTask", TaskParameter::stack_size, NULL,
        TaskParameter::priority_max - 1, &task_handle_real_time_, 1);
    xTaskCreatePinnedToCore(wiFiTask, "wiFiTask", TaskParameter::stack_size, NULL,
        TaskParameter::priority_min, &task_handle_wifi_, 1);
    xTaskCreatePinnedToCore(cameraTask, "cameraTask", TaskParameter::stack_size, NULL,
        TaskParameter::priority_min, &task_handle_camera_, 1);

    /*
     *  Attach the timer interrupt handler to the interrupt timer.
     *  See the interrupt header for details.
     */
    float freq = static_cast<float>(1 / PeriodParameter::fast);

    timer_->setFrequency(freq, timerInterruptHandler);


    /*
     *  Print initialization status to serial based on
     *  the current worst log level.
     */
    if (biped::Serial::getLogLevelWorst() <= LogLevel::error)
    {
        biped::Serial(LogLevel::warn) << "Initialized with error(s).";
    }
    else
    {
        biped::Serial(LogLevel::info) << "Initialized.";
    }
}

/**
 *  @brief  Main program loop task function.
 *
 *  This function is called by the loop task created and launched by
 *  the ESP-IDF framework. The loop task has a low priority and calls
 *  the best-effort task function.
 */
void
loop()
{
    /*
     *  Perform best-effort tasks.
     */
    bestEffortTask();

}
