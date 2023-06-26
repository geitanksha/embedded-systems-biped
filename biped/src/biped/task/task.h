/**
 *  @file   task.h
 *  @author Simon Yu
 *  @date   12/03/2022
 *  @brief  Task function header.
 *
 *  This file defines the task functions.
 */

/*
 *  Include guard.
 */
#ifndef TASK_TASK_H_
#define TASK_TASK_H_

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  I/O expander A interrupt task function.
 *
 *  This function handles I/O expander A interrupts. The
 *  task has the highest priority but may be preempted by
 *  any tasks with the same priority or by priority
 *  inheritance. The function goes to sleep until woken.
 */
void
ioExpanderAInterruptTask(void* pvParameters);

/**
 *  @brief  I/O expander B interrupt task function.
 *
 *  This function handles I/O expander B interrupts. The
 *  task has the highest priority but may be preempted by
 *  any tasks with the same priority or by priority
 *  inheritance. The function goes to sleep until woken.
 */
void
ioExpanderBInterruptTask(void* pvParameters);

/**
 *  @brief  Camera task function.
 *
 *  This function streams camera images via an HTTP
 *  web server. The camera task has the lowest priority
 *  and may be preempted by any tasks with the same or
 *  higher priorities. The function goes to sleep until
 *  woken.
 */
void
cameraTask(void* pvParameters);

/**
 *  @brief  Wi-Fi task function.
 *
 *  This function initializes the Wi-Fi driver. The
 *  Wi-Fi task has the lowest priority and may be
 *  preempted by any tasks with the same or higher
 *  priorities.
 */
void
wiFiTask(void* pvParameters);

/**
 *  @brief  Real-time task function.
 *
 *  This function performs the real-time task with
 *  strict timing deadlines. There are two timing
 *  domains, a fast domain and a slow domain. The
 *  real-time task has a high priority but may be
 *  preempted by any tasks with the same or higher
 *  priorities or by priority inheritance. The
 *  function goes to sleep until woken.
 *
 *  This function periodically executes the Biped
 *  software stack, which contains three main stages:
 *  sensing, control, and actuation. The stack first
 *  performs sensing which gathers data from sensors.
 *  Then, the stack executes the controller which
 *  retrieves the sensor data and produces an actuation
 *  command. Lastly, the actuator object takes the
 *  actuation command and performs the actuation.
 */
void
realTimeTask(void* pvParameters);

/**
 *  @brief  Best-effort task function.
 *
 *  This function performs the best-effort or
 *  non-real-time task that do not require strict
 *  timing deadlines. The function is called by the
 *  main program loop task function. The best-effort
 *  task has a low priority and may be preempted by
 *  any tasks with the same or higher priorities
 *  or by priority inheritance.
 */
void
bestEffortTask();
}   // namespace biped

#endif  // TASK_TASK_H_
