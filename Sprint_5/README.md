Overview:

This sprint extends the earlier FreeRTOS project by adding a dedicated logging task and synchronization between tasks. Sensor, actuator, and control tasks can produce messages, while the logging task collects these messages from a queue and sends them to the PC through UART. A binary semaphore signals when new log data is available, and a mutex protects UART access so transmissions do not interfere with one another.
The work supports the overall embedded project by making its sensor readings and actuator states easier to monitor. Separating message creation from UART output also provides a clearer structure for a multi-task application as more functions are added.

Implementation:
1. Cloned sprint 4 part 2 to build upon it

2. Created a logging task using osthreadnew(). created a binary semaphore handle using semaphoreCreateBinary() and another queue to store the messages. Created a helper function called logsend() to log messages in the queue called in the sensor and actuator task and wakign the semaphore using xSemaphoreGive(). in the logging task, if the xSemaphoreTake() passes ie the log is available, to print the log using uart.(Part 1)

3. Created a  mutex, logMutex is created with xSemaphoreCreateMutex(). Before writing a log message to UART, the logging task takes the mutex. It releases the mutex after transmission is complete. This ensures that only one task at a time uses the shared UART output, reducing the risk of overlapping or corrupted messages. In addition to this helper function were created to move majority of the logic out of the sensor/control/actuator tasks and clean the code up. Also to align with the sprint requirements I converted all osthreadnew() to xtaskcreate() after I got it working wiht this application.(Part 2)

Results:
1. The result of Part 1 is a combined output of the temp reading and the fan speed % in the uart

2. The result of the Part 2 is a combined output of all uart transmition including the initalization prints and the temp and fan speed print, all uart prints are done through the logging tasks protected by the mutex.
