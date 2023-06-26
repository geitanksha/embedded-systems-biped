/**
 *  @file   interrupt.h
 *  @author Simon Yu
 *  @date   12/03/2022
 *  @brief  Interrupt function header.
 *
 *  This file defines the interrupt functions.
 */

/*
 *  Include guard.
 */
#ifndef TASK_INTERRUPT_H_
#define TASK_INTERRUPT_H_

/*
 *  External headers.
 */
#include <cstdint>
#include <esp_attr.h>

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  Attach an interrupt handler to the given pin.
 *
 *  This function attaches the given interrupt handler to
 *  the given pin with the given mode. Unlike its
 *  corresponding counterpart in the Arduino framework,
 *  this function does not install GPIO interrupt service.
 */
void
attachInterrupt(const uint8_t& pin, void
(*handler)(void), const int& mode);

/**
 *  @brief  Perform a digital read from the given pin.
 *
 *  This function performs a digital read from the given pin
 *  and returns the state of the pin read. Unlike its
 *  corresponding counterpart in the Arduino framework,
 *  this function is safe to be called from an interrupt
 *  context.
 */
int IRAM_ATTR
digitalReadFromISR(uint8_t pin);

/**
 *  @brief  Left encoder A interrupt handler.
 *
 *  This function handles left encoder A interrupts.
 *  The function calls the corresponding callback member
 *  function in the sensor class.
 */
void IRAM_ATTR
encoderLeftAInterruptHandler();

/**
 *  @brief  Left encoder B interrupt handler.
 *
 *  This function handles left encoder B interrupts.
 *  The function calls the corresponding callback member
 *  function in the sensor class.
 */
void IRAM_ATTR
encoderLeftBInterruptHandler();

/**
 *  @brief  Right encoder A interrupt handler.
 *
 *  This function handles right encoder A interrupts.
 *  The function calls the corresponding callback member
 *  function in the sensor class.
 */
void IRAM_ATTR
encoderRightAInterruptHandler();

/**
 *  @brief  Right encoder B interrupt handler.
 *
 *  This function handles right encoder B interrupts.
 *  The function calls the corresponding callback member
 *  function in the sensor class.
 */
void IRAM_ATTR
encoderRightBInterruptHandler();

/**
 *  @brief  I/O expander A interrupt handler.
 *
 *  This function handles all interrupts on I/O expander A.
 *  The function wakes the I/O expander A interrupt task.
 */
void IRAM_ATTR
ioExpanderAInterruptHandler();

/**
 *  @brief  I/O expander B interrupt handler.
 *
 *  This function handles all interrupts on I/O expander A.
 *  The function wakes the I/O expander A interrupt task.
 */
void IRAM_ATTR
ioExpanderBInterruptHandler();

/**
 *  @brief  Push button A interrupt handler.
 *
 *  This function handles push button A interrupts.
 *  The function starts the planner's plan.
 */
void IRAM_ATTR
pushButtonAInterruptHandler();

/**
 *  @brief  Push button B interrupt handler.
 *
 *  This function handles push button B interrupts.
 *  The function calls the corresponding callback member
 *  function in the sensor class.
 */
void IRAM_ATTR
pushButtonBInterruptHandler();

/**
 *  @brief  Timer interrupt handler.
 *
 *  This function handles timer interrupts. The function
 *  wakes the real-time task.
 */
bool IRAM_ATTR
timerInterruptHandler(void* timer);
}   // namespace biped

#endif  // TASK_INTERRUPT_H_
