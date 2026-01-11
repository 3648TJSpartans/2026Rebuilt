# 3648 Skeleton Repository
This is our team's template repository for future First Robotics Competitions. It contains most of the basics for our swerve drive bots, including AprilTag localization, Swerve, LEDs, Vision, and networking.

## Naming Conventions
### Logging
Logging should be as broken down as possible and as frequent as possible. Every variable should be readable.

* Subsystems should be under the banner _Subsystem_.
* Commands under the _Commands_.
* Debug information under _Debug_.
* Utils under _Util_.

The folder should in turn be followed by the class name, followed by the method (for particularly complcated systems), followed by the variable logged.  
__E.g. to log Drive.getPose():__

`Logger.recordOutput("Subsystems/Drive/Pose", Drive.getPose());`
### Variable Naming
Constants should have the prefix _k_. Instance variables are given a prefix *m_*.

__Example Constants:__
```
public static final double kOdometryFrequency = 100.0;
public static final double kTrackWidth = Units.inchesToMeters(24.8);
```

__Example Instance Variables__
```
private boolean m_firstTimeSlot;
private boolean m_onShift;
private double m_timeLeft;
private double m_timeUntil;
private double m_time;

public ShiftTracker(){
    m_firstTimeSlot = false;
    m_onShift = false;
    m_time = 0.0;
}
```
## Trajectory Calculations - Desmos
Math used to find an initial velocity $v_0$ and angle $\theta$ to land within the Hub. Click the image to access tool. 
[![Desmos Tool](src/main/resources/readMe/DesmosTool.png)](https://www.desmos.com/calculator/2vgcp7eqsk)
