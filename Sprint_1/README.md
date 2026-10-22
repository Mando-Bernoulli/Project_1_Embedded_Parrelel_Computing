Overview:
Sprint 1 introduces the STM32 development workflow and establishes the software and programming foundations for later embedded-system work. The activities progress from installing the tools, to controlling a GPIO output, to transmitting serial messages, and then to running a simple LED task under an RTOS. A bonus activity combines LED state changes with UART status messages and a configurable delay.
The sprint's purpose is to verify that the development board can be programmed and debugged from the PC, that firmware can interact with hardware, and that the MCU can communicate observable status information to a host computer. These are basic building blocks for larger projects that later combine sensors, control logic, task scheduling, and serial logging.

Implementation:
1. first we download and setup the software which is STM32CUBEIDE and STM32CUBEMX as well as identifying the microcontroller we are working on which is the NUCLEO-L476RG
   
2. The bare-metal exercise uses the STM32 HAL to change a GPIO output at a one-second interval. The GPIO pin must be configured as an output in the project configuration or initialization code. The main loop then toggles the pin and delays before repeating. This task introduces the firmware cycle of initialization followed by a continuous loop (Blinky)

3. A separate project is used to configure serial output from the MCU to the PC. The UART peripheral needs a baud rate and frame format that match the terminal application, and the board's UART pins must be routed correctly. Firmware can send a fixed message using a HAL transmit function. Seeing the message in a serial terminal provides evidence that the UART peripheral, physical connection, and PC terminal settings are compatible.(UART)
