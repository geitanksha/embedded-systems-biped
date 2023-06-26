# Biped

## Table of Contents

1. [Project Setup](#project-setup)
2. [The Biped](#the-biped)
3. [Getting Started](#getting-started)
4. [Lab 6: Sensing and Actuation](#lab-6-sensing-and-actuation)
5. [Lab 7: Control](#lab-7-control)
6. [Lab 8: Planning](#lab-8-planning)
7. [Lab 9: Applications](#lab-9-applications)

## Project Setup

This project depends on the installation of the following system packages:

```bash
curl, tar, unzip, cmake, make, doxygen, python3-serial
```

After cloning the project, navigate to the project root directory.

To set up the project, run the setup script as follows:

```bash
./scripts/biped/setup.bash
```

During the setup process, pay close attention to the output of the setup script and check for any errors.

Doxygen will be your best friend for this project. To generate the Doxygen documentation for the project, run the Doxygen script as follows:

```bash
./scripts/biped/doxygen.bash
```

The generated Doxygen documentation would be located in a new `html` directory under `doc/doxygen`. To open the documentation, go to `doc/doxygen/html` and open the `index.html` file using a web browser. Please remember, whenever in doubt, always search in Doxygen first. Refer to the [Doxygen Manual](https://www.doxygen.nl/manual/index.html) for more details.

## The Biped

The Biped is a custom two-wheeled self-balancing robot designed by the CS 431 staff with various advanced sensors onboard, such as motor encoders for measuring wheel movements, inertial measurement units (IMUs) for measuring accelerations, attitudes, and angular velocities, time-of-flight sensors for measuring distances to surroundings, etc. The Biped also employs a small but capable microcontroller unit (MCU), the ESP32, with a dual-core processor and built-in wireless capabilities. Last but not least, the Biped also provides enormous rooms for expansions with two dual-port digital I/O expanders and a built-in breadboard for rapid prototyping.

There are a couple of things to be aware of while using Biped:
1. Due to the limited number of Bipeds we have, the Bipeds cannot be removed from the lab at any given time.
2. Please be careful not to drop the Biped and always perform any planner-related experiments on the floor.
3. It is always safer to place the Biped upside down while uploading the program.
4. Notify the TA immediately of any loose components, as loose parts might impact the control performance.
5. Contact the TA for any other hardware-related issues.

## Getting Started

First, navigate to the project root directory.

To build the project, perform the following:

```bash
cd build/biped
make
```

To upload the built project to the Biped, connect the USB-C cable, and perform the following in the same build directory:

```bash
make upload SERIAL_PORT=/dev/ttyUSB0
```

The serial port might differ depending on the operating system.

All lab objectives and instructions are in the form of comment blocks in the header and source files. Please RTDC (Read the Docs Carefully) as the comment blocks contain crucial information for understanding and completing the lab.

Some lab objectives might require prolonged testing, experiments, and debugging. Therefore, please start early!

There may be some warnings when building the project for the first time. Those warnings are caused by the incomplete codebase yet to be finished by you. As you progress through the labs, the warnings would gradually go away.

If you finished a lab early, feel free to get a head start on the next lab. However, please make sure you are ready for the demos for the current lab.

The Biped labs are brand-new. Therefore, there are likely rough spots throughout the codebase. Please help us improve by reporting bugs, typos, or any other issues to the TA.

Good luck, and have fun!

## Lab 6: Sensing and Actuation

In this lab, you will implement Biped's sensing and actuation, i.e., reading data from the sensors and performing motor actuation. Additionally, you will be setting up various framework-related components, such as object instantiations, interrupt handling, FreeRTOS task setup, etc.

### Objectives

1. Implement all Lab 6 todos in `actuator.cpp`.
2. Implement all Lab 6 todos in `biped.cpp`.
3. Implement all Lab 6 todos in `compass.cpp`.
4. Implement all Lab 6 todos in `encoder.cpp`.
5. Implement all Lab 6 todos in `imu.cpp`.
6. Implement all Lab 6 todos in `interrupt.cpp`.
7. Implement all Lab 6 todos in `sensor.cpp`.
8. Implement all Lab 6 todos in `task.cpp`.

RTDC the comment blocks in the above header and source files for detailed steps and instructions. Search for `// TODO LAB 6 YOUR CODE HERE.` in the above header and source files to locate the todos.

### Demo [100 pts]

In the `bestEffortTask` function, demonstrate the correctness of your sensor data acquisition by showing the values of the following structs:
1. `EncoderData` struct.
2. BMX160 `IMUData` struct.
3. MPU6050 `IMUData` struct.
4. `TimeOfFlightData` struct.

Please show all angle values in degrees. Show the struct values using either the OLED display or the serial connection. For the serial connection, add a small delay after the serial prints for better readability, but do remember to remove the delay later.

1. [20 pts] Demonstrate the correctness of all encoder data by pushing the Biped on a surface along its X-axis. The magnitude and the signedness of all data should match the definition of the standard body reference frame. For linear velocities, roughly check the magnitude.
2. [20 pts] Demonstrate the correctness of all IMU data, excluding compass and Z attitude data, from both IMUs by tiling the Biped by hand. The magnitude and the signedness of all data should match the definition of the standard body reference frame. For angular velocities, roughly check the magnitude.
3. [20 pts] Demonstrate the correctness of the compass and Z attitude data from the BMX160 IMU by rotating the Biped around its Z-axis by hand. The magnitude and the signedness of all data should match the definition of the standard body reference frame. The compass and Z attitude data might not be accurate due to the lack of compass calibration at this point; however, the data should not drift with the Biped being at the same location.
4. [20 pts] Demonstrate the correctness of all range data from all time-of-flight sensors by placing objects in front of the Biped. The magnitude of the range data should roughly reflect the distance in meters from the sensors to the object.

Place the Biped upside-down. In the `bestEffortTask` function, demonstrate the correctness of your motor actuation by creating and initializing an `ActuationCommand` struct, and then actuate the motors using the actuator object and the `ActuationCommand` struct.

5. [20 pts] Demonstrate that you can control both motors' spinning direction and speed by adjusting the motor directions and the pulse width modulation (PWM) values in the `ActuationCommand` struct. The Biped should move forward with PWM values above the minimum PWM value and vice versa.

If one fails to complete a demo, partial credits will be given by examining each todo's completion and correctness.

## Lab 7: Control

In this lab, you will implement Biped's controller. The controller of the Biped is responsible for its balancing and movements. Furthermore, you will perform compass calibration using an open-loop controller.

Note that the controller gains you would be tuning in this lab would be specific to only the Biped you tuned the gains on. Each Biped would have slightly different gain values due to the subtle differences in their hardware construction, e.g., the center of gravity. Therefore, from this point on, pick a Biped and stick to the same one (check for its serial number) for the remaining labs. If you notice any loose components, notify the TA immediately, as loose parts might affect the control performance.

### Objectives

1. Implement all Lab 7 todos in `biped.cpp`.
2. Implement all Lab 7 todos in `compass.cpp`.
3. Implement all Lab 7 todos in `controller.cpp`.
4. Implement all Lab 7 todos in `imu.cpp`.
5. Implement all Lab 7 todos in `interrupt.cpp`.
6. Implement all Lab 7 todos in `open_loop_controller.cpp`.
7. Implement all Lab 7 todos in `parameter.h`.
8. Implement all Lab 7 todos in `pid_controller.cpp`.
9. Implement all Lab 7 todos in `sensor.cpp`.
10. Implement all Lab 7 todos in `task.cpp`.

RTDC the comment blocks in the above header and source files for detailed steps and instructions. Search for `// TODO LAB 7 YOUR CODE HERE.` in the above header and source files to locate the todos.

### Demo [100 pts]

In the `bestEffortTask` function, demonstrate the correctness of your compass calibration by showing the compass data in the BMX160 `IMUData` struct.

Show the compass data using either the OLED display or the serial connection.

1. [25 pts] Demonstrate that your Biped can balance itself with adequate disturbance rejection (compensates light to medium push to the Biped chassis along the X-axis.)
2. [25 pts] Demonstrate that your Biped can return to its initial position when pushed.
3. [25 pts] Demonstrate the correctness of the compass calibration. The compass calibration should start after pressing push button B. The Biped should rotate 360 degrees or more clockwise and counterclockwise around its Z-axis during the calibration time. After the calibration, the X and Y compass data should be roughly within the calibration range and roughly centered around 0 when the Biped is rotated around its Z-axis by hand.
4. [25 pts] Demonstrate that your Biped can return to its initial Z attitude when pushed and that the Z attitude does not drift significantly.

If one fails to complete a demo, partial credits will be given by examining each todo's completion and correctness.

## Lab 8: Planning

In this lab, you will implement a waypoint-based planner and a maneuver-based planner. Additionally, you will fine-tune your controllers using the planners.

Remember to use the same Biped you used in the previous labs. Otherwise, your gains might not work. If you notice any loose components, notify the TA immediately, as loose parts might affect the control performance.

### Objectives

1. Implement all Lab 8 todos in `controller.cpp`.
2. Implement all Lab 8 todos in `interrupt.cpp`.
3. Implement all Lab 8 todos in `maneuver_planner.cpp`.
4. Implement all Lab 8 todos in `parameter.h`.
5. Implement all Lab 8 todos in `task.cpp`.
6. Implement all Lab 8 todos in `waypoint_planner.cpp`.

RTDC the comment blocks in the above header and source files for detailed steps and instructions. Search for `// TODO LAB 8 YOUR CODE HERE.` in the above header and source files to locate the todos.

### Demo [100 pts]

1. [50 pts] Demonstrate that your Biped can correctly execute the waypoint planner example plan. It is alright if the Z attitude of the Biped drifts during the plan due to nearby magnetic fields. The Biped should move forward and in reverse with a moderate speed and roughly turn to the Z attitude controller references specified by the plan, if any.
2. [50 pts] Demonstrate that your Biped can correctly execute the maneuver planner example plan. It is alright if the Z attitude of the Biped drifts during the plan due to nearby magnetic fields. The Biped should move forward and in reverse with a moderate speed and roughly turn to the Z attitude controller references specified by the plan, if any.

If one fails to complete a demo, partial credits will be given by examining each todo's completion and correctness.

## Lab 9: Applications

You have achieved so much at this point. It is time to have some fun! At this point, there are no further implementation-related objectives. Instead, you will enjoy using the codebase you built and focus on its applications. In this lab, create your own custom waypoint-based and maneuver-based plans for your Biped and instruct it to accomplish any meaningful tasks such as obstacle avoidance, object following, etc. Optionally, stream the camera images from the Biped to your computer while the Biped is in operation.

Remember to use the same Biped you used in the previous labs. Otherwise, your gains might not work. If you notice any loose components, notify the TA immediately, as loose parts might affect the control performance.

### Objectives

1. Implement all Lab 9 todos in `maneuver_planner.cpp`.
2. Implement all Lab 9 todos in `waypoint_planner.cpp`.

RTDC the comment blocks in the above header and source files for detailed steps and instructions. Search for `// TODO LAB 9 YOUR CODE HERE.` in the above header and source files to locate the todos.

### Optional Objectives: Camera

1. Implement all Lab 9 todos in `task.cpp`.

Stream the camera images from the Biped to your computer while the Biped is in operation. The Biped connects to the `IllinoisNet_Guest` Wi-Fi network by default. Restart the Biped if it fails to connect to the Wi-Fi network. Once connected, the Biped's IP address is printed onto its OLED display. Then, it is required to register the Biped's MAC address with the university using [this portal](https://clearpasspub.techservices.illinois.edu/guest/auth_login.php). The MAC address of the Biped is printed to the terminal during `make upload`.

Then, follow the steps below to register your Biped's MAC address on the portal:
1. Log in to the portal using your NetID and password.
2. Click on `Create new device`.
3. Enter the Biped's MAC address into the `* MAC Address:` field.
4. Enter anything into the `* Device Name:` field.
5. Check the `Enable AirGroup` check box.
6. Enter your and your groupmates' NetIDs into the `* Shared With:` field separated by commas.
7. Make sure the `Account Activation:` dropdown menu is selected to be `Now`
8. Accept the `Terms of Use` and click `Create` at the bottom.

After the registration, restart your Biped, note its IP address, and type it into a web browser. After a while, you should see the camera images being streamed to the webpage on your computer. Note that the webpage must be accessed with a computer connected to the `IllinoisNet` network using any of the NetIDs in the `* Shared With:` field back during the registration; otherwise, the university firewall might block the connection.

### Demo [100 pts]

1. [50 pts] Demonstrate your custom waypoint-based plans. Your plans should include waypoints that move the Biped forward, reverse, forward while turning right, forward while turning left, reverse while turning right, and reverse while turning left.
2. [50 pts] Demonstrate your custom maneuver-based plans. Your plans should enable the Biped to accomplish a meaningful task. Some examples of a meaningful task include but are not limited to obstacle detection and avoidance, i.e., detecting and going around an object, or object following, i.e., following a moving object.

Have fun and get creative on this lab!

If one fails to complete a demo, partial credits will be given by examining each todo's completion and correctness.
