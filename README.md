# RTOS_Data_Logger

This project is a revision project to implement the concepts that are learned from the course Mastering RTOS: Hands-on FreeRTOS and STM32Fx with Debugging. This project is not related to any project from the course but more to improve the comfort and fluency in RTOS.

Project Goal:  The primary goal is to create a system that reads data from multiple simulated sensors / one multi-sensor, processes the data, and stores the data persistently (simulated flash memory).  This project emphasizes data management, task synchronization, and resource management within an RTOS environment.

List of Anticipated Tasks:

1. Sensor Simulation Tasks:
   * Create separate tasks for each simulated sensor (e.g., temperature, pressure, humidity).
   * Each task periodically reads the sensor value (this could be a random number generator for simulation purposes).
   * The sensor reading tasks place the acquired data into a FreeRTOS Queue.  A separate queue should ideally be used for each sensor type, but this could be simplified for learning purposes.

2. Data Processing Task (Optional):
   * This task receives data from the sensor queues.
   * It performs any necessary processing, such as:
      * Averaging multiple readings.
      * Filtering noise.
      * Converting raw data to engineering units.
   * The processed data is then placed into another queue destined for the storage task.

3. Data Storage Task:
   * This task receives processed data from the queue (or directly from the sensor queues if there's no processing task).
   * It simulates writing the data to "flash memory."  In a real system, this would involve interacting with flash memory hardware. For simulation, you can write to a file on your host computer.
   * This task needs to handle potential "flash memory" access conflicts.  If another task (unlikely in this example, but good practice) tried to access the "flash" while the storage task was writing, data corruption could occur.

4. Data Display:
   * This task will receive the data from the sensor(s) after being processed by the Data Processing Task. Then, the data will be displayed on the LCD screen.
  
Hardware:
- STM32F407 Discovery board.
- Sensor BME680.
- Joy-it com-LCD20x4-B Display module.

Potential FreeRTOS Concept:
- Task Management: Creation, Suspension, Priorities, etc.
- Queues: a place to share data for Storage action, Display action, and Measuring. 1 queue between Sensor measuring and Data processing and 1 queue between Data processing, Data storing, and Data displaying.
- Software Timer: to trigger periodic tasks of measuring data and updating displayed content.
- Mutexes / Semaphores: protect the share value such as queue or storage.
- Synchronization: after the sensors measure the data, signal the processing task.
