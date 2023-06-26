/**
 *  @file   task.cpp
 *  @author Simon Yu
 *  @date   12/03/2022
 *  @brief  Task function source.
 *
 *  This file implements the task functions.
 */

 /*
  *  External headers.
  */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>

  /*
   *  Project headers.
   */
#include "actuator/actuator.h"
#include "platform/camera.h"
#include "controller/controller.h"
#include "platform/display.h"
#include "platform/serial.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "planner/maneuver_planner.h"
#include "platform/neopixel.h"
#include "sensor/sensor.h"
#include "task/task.h"
#include "planner/waypoint_planner.h"

   /*
    *  Biped namespace.
    */
namespace biped
{
    void
        ioExpanderAInterruptTask(void* pvParameters)
    {
        for (;;)
        {
            /*
             *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
             *  function. Set clear count on exit to true and maximum
             *  task wait time to be maximum delay.
             */
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            /*
             *  Validate I/O expander A object pointer and call the I/O
             *  expander interrupt callback function.
             *  See the I/O expander class for details.
             */
            if (io_expander_a_) {
                io_expander_a_->onInterrupt();
            }
        }

        /*
         *  Delete task upon exit using the FreeRTOS
         *  vTaskDelete function.
         */
        vTaskDelete(nullptr);
    }

    void
        ioExpanderBInterruptTask(void* pvParameters)
    {
        for (;;)
        {
            /*
             *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
             *  function. Set clear count on exit to true and maximum
             *  task wait time to be maximum delay.
             */
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            /*
             *  Validate I/O expander B object pointer and call the I/O
             *  expander interrupt callback function.
             *  See the I/O expander class for details.
             */
            if (io_expander_b_) {
                io_expander_b_->onInterrupt();
            }
        }

        /*
         *  Delete task upon exit using the FreeRTOS
         *  vTaskDelete function.
         */
        vTaskDelete(nullptr);
    }

    void
        cameraTask(void* pvParameters)
    {
        /*
         *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
         *  function. Set clear count on exit to true and maximum
         *  task wait time to be maximum delay.
         */

         /*
          *  Validate camera object pointer and perform streaming.
          *  See the camera class for details.
          */

          /*
           *  Delete task upon exit using the FreeRTOS
           *  vTaskDelete function.
           */
        vTaskDelete(nullptr);
    }

    void
        wiFiTask(void* pvParameters)
    {
        /*
         *  Initialize the Wi-Fi driver object and disable sleep.
         *  See parameter header for details
         */

         /*
          *  Delete task upon exit using the FreeRTOS
          *  vTaskDelete function.
          */
        vTaskDelete(nullptr);
    }

    void
        realTimeTask(void* pvParameters)
    {
        /*
         *  Declare start time point and set to 0.
         */
        unsigned long time_point_start = 0;

        for (;;)
        {
            /*
             *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
             *  function. Set clear count on exit to true and maximum
             *  task wait time to be maximum delay.
             */
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            /*
             *  Calculate real-time task interval and update start
             *  time point.
             */
            interval_real_time_task_ = micros() - time_point_start;
            time_point_start = micros();

            /*
             *  Perform fast domain sensing.
             *  See the sensor class for details.
             */
            sensor_->sense(true);

            /*
             *  Perform fast domain control.
             *  See the controller class for details.
             */
            controller_->control(true);

            /*
             *  Slow domain tasks.
             */
            if (timer_domain_ >= PeriodParameter::slow)
            {
                /*
                 *  Perform slow domain sensing.
                 *  See the sensor class for details.
                 */
                sensor_->sense(false);

                /*
                 *  Perform slow domain control.
                 *  See the controller class for details.
                 */
                controller_->control(false);

                /*
                 *  Reset timing domain timer.
                 */
                timer_domain_ = 0;
            }

            /*
             *  Perform actuation using the actuation
             *  command struct from the controller object.
             *  See the actuator class for details.
             */

            actuator_->actuate(controller_->getActuationCommand());
            /*
             *  Update timing domain timer.
             */
            timer_domain_ += PeriodParameter::fast;

            /*
             *  Calculate real-time task execution time.
             */
            execution_time_real_time_task_ = micros() - time_point_start;
        }

        /*
         *  Delete task upon exit using the FreeRTOS
         *  vTaskDelete function.
         */
        vTaskDelete(nullptr);
    }

    void
        bestEffortTask()
    {
        /*
         *  Declare camera task woken flag and set to false.
         */
        static bool camera_task_woken = false;

        /*
         *  Print serial number to display.
         */
        Display(0) << "Biped: #" << serial_number_;

        /*
         *  Print real-time task timings to display.
         */
        Display(1) << "Real-time: " << execution_time_real_time_task_ << " "
            << interval_real_time_task_;

        /*
         *  Print controller active status to display.
         */
        if (controller_->getActiveStatus())
        {
            Display(2) << "Controller: active";
        }
        else
        {
            Display(2) << "Controller: inactive";
        }

        /*
         *  Execute plan and store the planner stage.
         *  See the planner class for details.
         */
        const int stage = planner_->plan();


        /*
         *  Print planner status to display.
         */
        if (stage < 0)
        {
            Display(3) << "Planner: inactive";
        }
        else
        {
            Display(3) << "Planner: stage " << stage;
        }

        /*
         *  Print Wi-Fi status to display.
         */
        if (WiFi.status() == WL_CONNECTED)
        {
            Display(4) << "Wi-Fi: " << WiFi.localIP().toString().c_str();

            /*
             *  If the Wi-Fi is connected, validate the camera task
             *  handle and wake camera task using the FreeRTOS
             *  xTaskNotifyGive function if the camera task woken flag
             *  is false. Then, set camera task woken flag to true.
             */
        }
        else
        {
            Display(4) << "Wi-Fi: disconnected";
        }
        EncoderData enc_d = sensor_->getEncoderData();
        IMUData bmx_d = sensor_->getIMUDataBMX160();
        IMUData mpu_d = sensor_->getIMUDataMPU6050();
        TimeOfFlightData tof_d = sensor_->getTimeOfFlightData();
        Compass::Calibration cal_d = sensor_->getCompassCalibrationBMX160();

        Display(5) << "compass x: " << bmx_d.compass_x;
        Display(6) << "compass y: " << bmx_d.compass_y;
        Display(7) << "compass z: " << bmx_d.compass_z;

        /* Encoder info display */
        // Display(1) << "x_scaler: " << cal_d.scaler_x;
        // Display(2) << "y_scaler: " << cal_d.scaler_y;
        // Display(3) << "z_scaler: " << cal_d.scaler_z;
        // Display(4) << "x_offset: " << cal_d.offset_x;
        // Display(5) << "y_offset: " << cal_d.offset_y;
        // Display(6) << "z_offset: " << cal_d.offset_z;


        /* BMX 160 info display */
        // biped::Serial(LogLevel::info) << "BMX160: acceleration_x " << bmx_d.acceleration_x;
        // biped::Serial(LogLevel::info) << "BMX160: acceleration_y " << bmx_d.acceleration_y;
        // biped::Serial(LogLevel::info) << "BMX160: acceleration_z " << bmx_d.acceleration_z;
        // biped::Serial(LogLevel::info) << "BMX160: attitude_x " << bmx_d.attitude_x;
        // biped::Serial(LogLevel::info) << "BMX160: attitude_y " << bmx_d.attitude_y;
        // biped::Serial(LogLevel::info) << "BMX160: attitude_z " << bmx_d.attitude_z;
        // biped::Serial(LogLevel::info) << "BMX160: ang_vel_x " << bmx_d.angular_velocity_x;
        // biped::Serial(LogLevel::info) << "BMX160: ang_vel_y " << bmx_d.angular_velocity_y;
        // biped::Serial(LogLevel::info) << "BMX160: ang_vel_z " << bmx_d.angular_velocity_z;
        // biped::Serial(LogLevel::info) << "BMX160: compass_x " << bmx_d.compass_x;
        // biped::Serial(LogLevel::info) << "BMX160: compass_y " << bmx_d.compass_y;
        // biped::Serial(LogLevel::info) << "BMX160: compass_z " << bmx_d.compass_z;
        // biped::Serial(LogLevel::info) << "BMX160: temperature " << bmx_d.temperature;


        /* MPU 6050 info display */
        // biped::Serial(LogLevel::info) << "MPU6050: acceleration_x " << mpu_d.acceleration_x;
        // biped::Serial(LogLevel::info) << "MPU6050: acceleration_y " << mpu_d.acceleration_y;
        // biped::Serial(LogLevel::info) << "MPU6050: acceleration_z " << mpu_d.acceleration_z;
        // biped::Serial(LogLevel::info) << "MPU6050: attitude_x " << mpu_d.attitude_x;
        // biped::Serial(LogLevel::info) << "MPU6050: attitude_y " << mpu_d.attitude_y;
        // biped::Serial(LogLevel::info) << "MPU6050: attitude_z " << mpu_d.attitude_z;
        // biped::Serial(LogLevel::info) << "MPU6050: ang_vel_x " << mpu_d.angular_velocity_x;
        // biped::Serial(LogLevel::info) << "MPU6050: ang_vel_y " << mpu_d.angular_velocity_y;
        // biped::Serial(LogLevel::info) << "MPU6050: ang_vel_z " << mpu_d.angular_velocity_z;
        // biped::Serial(LogLevel::info) << "MPU6050: compass_x " << mpu_d.compass_x;
        // biped::Serial(LogLevel::info) << "MPU6050: compass_y " << mpu_d.compass_y;
        // biped::Serial(LogLevel::info) << "MPU6050: compass_z " << mpu_d.compass_z;
        // biped::Serial(LogLevel::info) << "MPU6050: temperature " << mpu_d.temperature;

        /* compass info display*/
        // biped::Serial(LogLevel::info) << "Compass values x: " << bmx_d.compass_x;
        // biped::Serial(LogLevel::info) << "Compass values y: " << bmx_d.compass_y;
        // biped::Serial(LogLevel::info) << "Compass values z: " << bmx_d.compass_z;
        /* Time of Flight info display */
        // biped::Serial(LogLevel::info) << "TOF: range_left " << tof_d.range_left;
        // biped::Serial(LogLevel::info) << "TOF: range_middle " << tof_d.range_middle;
        // biped::Serial(LogLevel::info) << "TOF: range_right " << tof_d.range_right;



        /*
         *  Show the NeoPixel frame.
         *  See the NeoPixel class for details.
         */
        neopixel_->show();

        /*
         *  Flush the display driver buffer to the display.
         *  See the display class for details.
         */
        Display::display();
    }
}   // namespace biped
