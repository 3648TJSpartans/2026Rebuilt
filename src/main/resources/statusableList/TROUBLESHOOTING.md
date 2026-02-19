# Troubleshooting with LEDS
Follow this guide to troubleshoot issues with subsystems using the LEDs.

## Reading Statuses
To read the status for a particular subsystem, count from the **RoboRIO end of the LED strip** to the LED that corresponds to your subsystem. See the subsystem names below for indices.

### Colors
| Color | Meaning | Description                                                               |
| ----- | ------- | ------------------------------------------------------------------------- |
| 🟩     | Good    | Subsystem fully operational.                                              |
| 🟪     | Warning | There is a minor issue with the subsystem that may impede its operation.  |
| 🟥     | Error   | There is a major issue with the subsystem that will impede its operation. |
| ⬜     | Unknown | Status reporting is yet to be implemented for the subsystem.              |

## Deciphering issues
The following is a list of what might be causing warning and errors for each subsystem. Check each problem that corresponds with the color to figure out what exactly is wrong.



### `1` Drive
| Color | Problem                    | Solution                                                                                                               |
| ----- | -------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| 🟥     | Gyro disconnected          | Ensure the gyroscope is plugged into the roboRIO and recieving power.                                                  |
| 🟥     | Swerve module disconnected | Ensure all swerve modules are plugged in and powered. Check AdvantageScope or see a coder if the issue isn't apparent. |

### `2` Vision
| Color | Problem                            | Solution                                                 |
| ----- | ---------------------------------- | -------------------------------------------------------- |
| 🟪     | Cameras haven't detected AprilTags | Face a camera towards an AprilTag to get a reading.      |
| 🟥     | Camera disconnected                | Ensure each camera is powered and connected to ethernet. |

### `3` Turret
| Color | Problem                           | Solution                                                                   |
| ----- | --------------------------------- | -------------------------------------------------------------------------- |
| 🟪     | Not homed                         | Rotate the turret to the home position.                                    |
| 🟪     | Hall effect sensor hasn't changed | Rotate the turret to ensure the sensor is working.                         |
| 🟥     | Motor disconnected                | Make sure the motor is connected to the CAN and the encoder is plugged in. |

### `4` Kicker
| Color | Problem                  | Solution                                                                   |
| ----- | ------------------------ | -------------------------------------------------------------------------- |
| 🟪     | IR sensor hasn't changed | Trigger the sensor to ensure it's working                                  |
| 🟥     | Motor Disconnected       | Make sure the motor is connected to the CAN and the encoder is plugged in. |

### `5` Shooter
| Color | Problem            | Solution                                                                   |
| ----- | ------------------ | -------------------------------------------------------------------------- |
| 🟥     | Motor Disconnected | Make sure the motor is connected to the CAN and the encoder is plugged in. |

### `6` Climber
| Color | Problem            | Solution                                                                   |
| ----- | ------------------ | -------------------------------------------------------------------------- |
| 🟥     | Motor Disconnected | Make sure the motor is connected to the CAN and the encoder is plugged in. |

### `7` Hood
| Color | Problem            | Solution                                                                   |
| ----- | ------------------ | -------------------------------------------------------------------------- |
| 🟥     | Motor Disconnected | Make sure the motor is connected to the CAN and the encoder is plugged in. |

### `8` Hopper/Indexer
| Color | Problem            | Solution                                                                   |
| ----- | ------------------ | -------------------------------------------------------------------------- |
| 🟥     | Motor Disconnected | Make sure the motor is connected to the CAN and the encoder is plugged in. |


### `9` Intake
| Color | Problem                         | Solution                                                                   |
| ----- | ------------------------------- | -------------------------------------------------------------------------- |
| 🟥     | Motor Disconnected              | Make sure the motor is connected to the CAN and the encoder is plugged in. |
| 🟥     | Pneumatic solenoid disconnected | Make sure the pneumatic assembly is connected properly                     |

### `10` Logging USB
| Color | Problem           | Solution                                                                                                                    |
| ----- | ----------------- | --------------------------------------------------------------------------------------------------------------------------- |
| 🟪     | USB almost full   | The USB drive is almost at full capacity. Replace it with a different one (formatted FAT32) or see a coder to clean it out. |
| 🟥     | USB not connected | Ensure the USB drive is plugged in. If it is, replug it and wait about 2 seconds.                                           |

### `11` Battery
| Color | Problem                  | Solution                                                  |
| ----- | ------------------------ | --------------------------------------------------------- |
| 🟪     | Battery under 12.5 volts | Be aware the battery may need to be replaced soon.        |
| 🟥     | Battery under 12 volts   | Replace the battery, or be aware drive may not work well. |






