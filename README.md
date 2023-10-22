# -Attendance-management-system-AMS-
An attendance management system (AMS) is a  tool that helps organizations track employee working hours. 
# -AMSs can track:
        1.Attendance, Breaks, Time off, 
        2.Arrival time, Duration of absence, 
        3.Clocking in and out
        4.Leave management
        5.Payroll integration
        6.Reporting
        7.Biometric attendance
        8.Calendar integration
# -When choosing an AMS, you can consider things like: 
        1.The number of people who work at your company.
        2.Keeping Track of each employee work hours.
# -STM32F401-FINGERPRINT-SENSOR  
          This repository contains sample codes for interfacing an stm32f401 microcontroller with a fingerprint sensor.  
# -Fingerprint sensor details  
        The fingerprint sensors use UART for communicating with a microcontroller.   
          Some popular fingerprint sensor modules include:      
          1. AS608 sensor  
          2. R305 sensor
          3. R307 sensor, etc.  
        The AS608 sensor was used in testing this library.      
          1.It requires a 3.3v power supply, and it is compatible with microcontrollers that operate with 3.3v logic level.  
          2.It has a default baud rate of 57600 for UART communication.  
          3.In order to communicate with the sensor, some data must be transmitted to the sensor by the microcontroller.
          4.Upon receiving data from the microcontroller, the fingerprint sensor gives a feedback (or response).
# -Sample codes  
        Three sample codes are provided in this repo in order to test the fingerprint sensor.  
        The test applications include:  
        1. Enrolling a finger  
        2. Scanning (or searching) for a fingerprint in the sensor's database  
        3. Deleting a specific fingerprint from the sensor's database.  
        4. Deleting all the data library buffer in sensor.
# -example image
        ![image](https://github.com/maybesravan/-Attendance-management-system-AMS-/assets/81691560/9e5e0d13-bfd6-4c97-ad8c-e38766003903)

