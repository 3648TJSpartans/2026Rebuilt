package frc.robot.commands.simpleMotorCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.simpleMotor.SimpleMotor;

public class SimpleMotorCmd extends Command {
    private final SimpleMotor m_simpleMotor;
    private final double speed;

    public SimpleMotorCmd(SimpleMotor simpleMotor, double speed) {
        m_simpleMotor = simpleMotor;
        this.speed = speed;
        addRequirements(m_simpleMotor);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Commands/simpleMotor/CommandScheduled", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("Commands/simpleMotor/Speed", speed);
        m_simpleMotor.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_simpleMotor.stop();
        Logger.recordOutput("Commands/simpleMotor/CommandScheduled", false);
    }

    @Override
    public boolean isFinished() {
        /*
         * Because this command lacks a finish state, it's important to make sure
         * it ends some other way, such as using whileTrue (which will make the command
         * end when the button is released) or a raceGroup, which finishes all commands
         * in the group when any command in the group finishes.
         */
        return false;
    }

}