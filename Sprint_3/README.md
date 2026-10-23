Overview:

This sprint extends the sensor-reading task from Sprint 2 by adding a separate control task. The sensor task periodically acquires a measurement; the control task reads that value, applies decision logic, and produces a trigger that can be shown through an LED or serial output. The sprint also introduces the idea of task switching in FreeRTOS.
The activity supports the project goal of separating data acquisition from decision-making. On a single-core STM32, tasks generally execute concurrently through scheduling and context switches rather than literally running at the same instant. This division makes the software easier to extend later with actuator and logging tasks.

Implementation:
1. Created a Copy of sprint 2 part 2 to build on the previous sensor task.
  
2. Created a task using osthreadnew() to create a control task with a lower priority than the sensor task. For this sprint, the two tasks share the latest sensor value through a global variable. The control task copies the shared value to a local variable and evaluates it against a project setpoint. To demonstrate the control task is working, the on board led was turned on when the temp reading was above the set point and back off when it was below (Part 1)
  
3. To show which task is active the traceTaskSwitch() was implemented to toggle the onboard LED and print the running task name over UART. This makes scheduler activity more visible during testing(Part 2)

Results:
1. The results for part 1 were as expected. the led turned on when the temp was above the set point of 26 degrees, the current temp was displayed in the tera term application.

2. The results for part 2 were as expected. The led was toggled and the current task was outputted to the tera term application alongside the current temp, since the control task also wrote to the same led pin as the task switched the signal would overlap.  
