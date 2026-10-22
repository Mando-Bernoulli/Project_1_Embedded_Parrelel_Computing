Overview:

This sprint establishes the sensor-acquisition foundation for the embedded project. The sprint asks for a temperature sensor to be read through I²C, with values displayed over UART, first in a conventional bare-metal loop and then from a native FreeRTOS task. The purpose is to validate the peripheral interface before adding the larger multi-task architecture.
The work fits the overall project by turning physical sensor data into a dependable software input. Once the sensor read function is reliable, later sprints can use the values for control decisions, actuator triggers, and logging. The provided handout uses a TMP3-style temperature-reading helper as its example, but the actual project sensor is the BME280 temp & humidity sensor

Implementation:
1. Created an STM32CubeMX/STM32CubeIDE project and configure the appropriate I²C SDA/SCL pins and UART pins. Checked for pin conflicts, confirmed the peripheral settings, saved the .ioc configuration, and generated the code.

2. Completed the sensor helper using the device address, temperature register, and output format defined by the datasheet. The function sends the register address over I²C, receives the measurement bytes, checks HAL status values, combines the bytes, handles signed values, and converts the result into engineering units. Called the helper from the main loop with a one second delay.(Part 1)

3. Made a copy of the bare-metal project, configured the HAL time base to use a hardware timer so the RTOS can use SysTick, and enabled FreeRTOS. Added a task with osthreadnew() that reads the sensor, reports the value through UART, and delays using vTaskDelay().(Part 2)

4. Modified the RTOS code to read temps from the BME280. Configured the initialization functions for the BME and created a read temp helper function. Modified the sensor read task to read temps from the sensor and output it through uart.(Part 3)

Results:
1. For Part 1 the temp was read form the TMP33 sensor and trasmitted to the tera term application and displayed the current sensor reading in a bare metal implentation

2. For Part 2 the temp was read form the TMP33 sensor and trasmitted to the tera term application and displayed the current sensor reading in a FreeRTOS implentation

3. For Part 3 the temp was read from the BME280 sensor and transmitted to the tera term application and displayed the current sensor reading in FreeRTOS

