# -Attendance-management-system-AMS-
An attendance management system (AMS) is a  tool that helps organizations track employee working hours. 
# -AMSs can track:
        1.Attendance, Breaks, Time off, 
        2.Arrival time, Duration of absence, 
        3.Leaves
        4.Clocking in and out
        5.Leave management
        6.Payroll integration
        7.Reporting
        8.Biometric attendance
        9.Missed clock-in alerts
        10.Calendar integration
# -When choosing an AMS, you can consider things like: 
        1.The number of people who work at your company
        2.Where your employees work from
        3.Whether it automatically applies company payroll rules
        4.Whether it generates the data you need to be in compliance with government audits
        5.The desired features and functionalities
        6.Whether the vendor ensures your data is protected  
# -STM32F103-FINGERPRINT-SENSOR  
          This repository contains sample codes for interfacing an stm32f103 microcontroller with a fingerprint sensor.  
# -Fingerprint sensor details  
          The fingerprint sensors use UART for communicating with a microcontroller.   
          Some popular fingerprint sensor modules include:      
          1. AS608 sensor  
          2. R305 sensor
          3. R307 sensor, etc.  
          The AS608 sensor was used in testing this library.      
          It requires a 3.3v power supply, and it is compatible with microcontrollers that operate with 3.3v logic level.  
          It has a default baud rate of 57600 for UART communication.  
          In order to communicate with the sensor, some data must be transmitted to the sensor by the microcontroller.  
          Upon receiving data from the microcontroller, the fingerprint sensor gives a feedback (or response).

# -Sample codes  
        Three sample codes are provided in this repo in order to test the fingerprint sensor.  
        The test applications include:  
        1. Enrolling a finger  
        2. Scanning (or searching) for a fingerprint in the sensor's database  
        3. Deleting a specific fingerprint from the sensor's database.  
        4. Deleting all the data library buffer in sensor.
