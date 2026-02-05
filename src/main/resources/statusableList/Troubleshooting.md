# Trouble Shooting with LEDS

The following can be used to trouble-shoot errors with the robot using our LED strip.

### Statuses

- 🟩 Good - Ready to start the match
- 🟨 Warning - There may be an issue with system functionality, please resolve. 
- 🟥 Error - The system will not be able to perform. Fix before a match. 
- 🟪 Unknown - This is yet to be coded. Let a coder know. 

### Troubleshooting
The following is a list of what might be causing warning and erros for the given subsystem and LED number.

While lights don't give the specific issue, the issue can be found via-log files. If nto quickly identifiable, find a coder. 

1. Drive
      - 🟥 Gyro Disconnected
        * Assure the gyroscope is plugged in and recieving power.
    - 🟥 Module Disconnected
      * Assure each module is powered and plugged in. 
      * If issue is not immediatly apparent, check AdvantageScope (see coder) to get the CAN ID.
2. Vision
    - 🟥 Camera Disconnected
      * Assure each camera is powered and connected via ethernet. If the issue is not immediately apparent, find a coder for torubleshooting. 
    - 🟨 Cameras have not seen April Tag
      * Face the robot towards an April Tag to get a valid reading. 
3. Turret
    - 🟥 Motor Disconnected
      * Make sure the motor CAN is connected and encoder is plugged in. 
    - 🟨 Not Homed
      * Put the turret in a homed position. 
    - 🟨 HAL Sensor hasn't changed
      * Change the state of the HAL sensor (triggered $\to$ untriggered or untriggered $\to$ triggered).
      * Do this by moving the turret. 
4. Kicker
    - 🟥 Motor Disconnected
      * Make sure the motor CAN is connected and encoder is plugged in. 
    - 🟨 IR Sensor unchanged
      * Trigger and un-trigger the IR sensor.
5. Shooter
      - 🟥 Motor Disconnected
      * Make sure the motor CAN is connected and encoder is plugged in. 
6. Climber
    - 🟥 Motor Disconnected
      * Make sure the motor CAN is connected and encoder is plugged in. 
7. Hood
    - 🟥 Motor Disconnected
      * Make sure the motor CAN is connected and encoder is plugged in. 
8. Hopper
    - 🟥 Motor Disconnected
      * Make sure the motor CAN is connected and encoder is plugged in. 
9. Intake
    - 🟥 Motor Disconnected
      * Make sure the motor CAN is connected and encoder is plugged in. 
    - 🟥 Solenoid Disconnected
      * Make sure the motor CAN is connected and encoder is plugged in. 
10. USB
    - 🟨 USB Disconnected
      * Unplug and replug in the USB stick. Wait 2 seconds. 
    - 🟨 USB Nearing full
      * The USb is almost out of storage. Replace with another stick. 
11. Battery
    - 🟨 Battery is under 12.5 Volts
    - 🟥 Battery is under 12.0 Volts
      * Replace battery





