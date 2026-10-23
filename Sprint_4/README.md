Overview:

This sprint changes the design from shared-global-variable communication to FreeRTOS queues. The project is structured around four logical tasks: sensor read, control, actuator, and logging. The main focus is to create the tasks using the native FreeRTOS API and to pass sensor readings from the sensor task to the control task, then pass control triggers from the control task to the actuator task.
Queue-based communication makes the flow of information between tasks more explicit and reduces direct dependence on shared global data. This is a step toward a modular embedded application in which sensing, decision-making, actuation, and logging can be developed and tested separately.

Implementation:
1. Clone Sprint 3 part 1 to build upon.

2. Created 2 queues using xQueueCreate(), sensor queue to send temp data from the sensor task to the control task and actuator queue to send data from the control task to the future actuator task. modified the sensor and control task to send data to the queue instead of the global variable.(Part 1)

3. Created an actuator task for a pc fan which included creating a new task using osthreadnew() as well as creating 2 helper function, fanset_speed() to write the PWM signal with the input duty cycle which is called in the actuator task and the fan_controlTemp() which outputs a duty cycle based on the predefined setpoints which is called in the control task. For the board to output a PWM signal, a new separate tim3 had to be activated and pin assigned to PA11 alongside the Tim6 which is being used by the FreeRTOS task. The control task was also modified to send the duty cycle value from the fan_controltemp() helper function to the actuator task. The UART output is the temp and fan speed in % for this implementation. an issue with this was the control task not being created on launch and the solution was to increase the default heap size in freeRTOSconfig.h header file to be able to accommodate a new task. the wiring for the pc fan was a 12 volt supply the PWM connection to the board(Part 2)

 Results:
 1. The result for part 1 was the same as sprint 3 part 2 as the led would turn on when the temp would be above the setpoint however this time using queues.

 2. The result for part 2 were the addition of the pc fan that would increase the duty cycle based on predetermined set points and the led would turn on when ever the fan is active, due to the make of the fan at 0% duty cycle the fan is enforced a minimum speed rather than turning off. the code worked as intended.
