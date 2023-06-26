/**
 *  @file   encoder.cpp
 *  @author Simon Yu
 *  @date   01/05/2023
 *  @brief  Encoder class source.
 *
 *  This file implements the encoder class.
 */

 /*
  *  External headers.
  */
#include <Arduino.h>

  /*
   *  Project headers.
   */
#include "platform/encoder.h"
#include "common/global.h"
#include "task/interrupt.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "platform/serial.h"

   /*
    *  Biped namespace.
    */
namespace biped
{
    Encoder::Encoder() : steps_left_(0), steps_right_(0)
    {
        /*
         *  Set pin mode for the motor encoder pins using
         *  the Arduino pin mode function. Use pull-up if the pin
         *  mode is input.
         *  See the parameter header for details.
         */
        pinMode(ESP32Pin::motor_left_encoder_a, INPUT_PULLUP);
        pinMode(ESP32Pin::motor_left_encoder_b, INPUT_PULLUP);
        pinMode(ESP32Pin::motor_right_encoder_a, INPUT_PULLUP);
        pinMode(ESP32Pin::motor_right_encoder_b, INPUT_PULLUP);

        /*
         *  Configure X velocity low-pass filter.
         */
        low_pass_filter_velocity_x_.setBeta(EncoderParameter::low_pass_filter_beta);
    }

    EncoderData
        Encoder::getData() const
    {
        /*
         *  Return the class member encoder data struct.
         */
        return this->data_;
    }

    void
        Encoder::read()
    {
        /*
         *  Take an average between the left and right total
         *  encoder step counters to be the overall encoder
         *  step count, convert the averaged overall encoder
         *  steps into meters, and populate the corresponding
         *  entries in the member encoder data struct.
         *
         *  See the parameter header for details.
         */
        double average_step = (steps_left_ + steps_right_) / 2.0;
        this->data_.steps = average_step;
        this->data_.position_x = average_step / EncoderParameter::steps_per_meter;
    }

    void
        Encoder::calculateVelocity()
    {
        /*
         *  Declare last overall encoder step counter and initialize to 0.
         */
        static long steps_last = 0;

        /*
         *  Read encoders.
         */
        read();

        /*
         *  Calculate the step count since the last overall encoder step
         *  counter update, convert the calculated steps into meters,
         *  calculate the X velocity using the slow domain period, filter
         *  the calculated X velocity using the low-pass filter, and
         *  populate the corresponding entry in the member encoder data struct.
         *
         *  See the parameter header for details.
         */
        double step_cnt_since_last = data_.steps - steps_last;
        double velocity = (step_cnt_since_last / EncoderParameter::steps_per_meter) / PeriodParameter::slow;
        this->data_.velocity_x = this->low_pass_filter_velocity_x_.filter(velocity);

        /*
         *  Set last overall encoder step counter to be the current
         *  overall encoder step count.
         */
        steps_last = static_cast<long>(data_.steps);
    }

    void IRAM_ATTR
        Encoder::onLeftA()
    {
        /*
         *  Read left encoder pin states using digitalReadFromISR
         *  function and increment/decrement the left encoder step
         *  counters based on the pin states read.
         *
         *  Incremental rotary encoder references:
         *  https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
         */
        int pin_a_state = digitalReadFromISR(ESP32Pin::motor_left_encoder_a);
        int pin_b_state = digitalReadFromISR(ESP32Pin::motor_left_encoder_b);
        if (pin_a_state != pin_b_state) {
            this->steps_left_ += 1;
        }
        else {
            this->steps_left_ -= 1;
        }
    }

    void IRAM_ATTR
        Encoder::onLeftB()
    {
        /*
         *  Read left encoder pin states using digitalReadFromISR
         *  function and increment/decrement the left encoder step
         *  counters based on the pin states read.
         *
         *  Incremental rotary encoder references:
         *  https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
         */
        int pin_a_state = digitalReadFromISR(ESP32Pin::motor_left_encoder_a);
        int pin_b_state = digitalReadFromISR(ESP32Pin::motor_left_encoder_b);
        if (pin_a_state != pin_b_state) {
            this->steps_left_ -= 1;
        }
        else {
            this->steps_left_ += 1;
        }
    }

    void IRAM_ATTR
        Encoder::onRightA()
    {
        /*
         *  Read right encoder pin states using digitalReadFromISR
         *  function and increment/decrement the right encoder step
         *  counters based on the pin states read.
         *
         *  Incremental rotary encoder references:
         *  https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
         */
        int pin_a_state = digitalReadFromISR(ESP32Pin::motor_right_encoder_a);
        int pin_b_state = digitalReadFromISR(ESP32Pin::motor_right_encoder_b);
        if (pin_a_state != pin_b_state) {
            this->steps_right_ -= 1;
        }
        else {
            this->steps_right_ += 1;
        }
    }

    void IRAM_ATTR
        Encoder::onRightB()
    {
        /*
         *  Read right encoder pin states using digitalReadFromISR
         *  function and increment/decrement the right encoder step
         *  counters based on the pin states read.
         *
         *  Incremental rotary encoder references:
         *  https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
         */
        int pin_a_state = digitalReadFromISR(ESP32Pin::motor_right_encoder_a);
        int pin_b_state = digitalReadFromISR(ESP32Pin::motor_right_encoder_b);
        if (pin_a_state != pin_b_state) {
            this->steps_right_ += 1;
        }
        else {
            this->steps_right_ -= 1;
        }
    }
}   // namespace biped
