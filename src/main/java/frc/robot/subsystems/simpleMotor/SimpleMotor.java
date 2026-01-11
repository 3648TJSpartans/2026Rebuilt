package frc.robot.subsystems.simpleMotor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * This subsystem controls a single motor with commands to spin
 * or stop it. Importantly, it lacks an encoder, which means that the 
 * robot won't know how much the motor has spun or its current position.
 */

/*
 * Subsystems are split into a main file, an IO, and a third file which houses
 * most of the logic. This file, the main file, receives an IO, and uses it
 * to structure the subsystem.
 */

public class SimpleMotor extends SubsystemBase {

    private final SimpleMotorIO io;

    public SimpleMotor(SimpleMotorIO io) {
        this.io = io;
    }

    /* Periodic, which runs constantly, is used here to log important values. */
    @Override
    public void periodic() {
        Logger.recordOutput("Subsystems/SimpleMotor/speed", io.getSpeed());
    }

    /* The remaining methods are all taken from the IO. */

    public void stop() {
        io.stop();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public double getSpeed(double speed) {
        return io.getSpeed();
    }
}
