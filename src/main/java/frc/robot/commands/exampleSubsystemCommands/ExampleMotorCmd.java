package frc.robot.commands.exampleSubsystemCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.exampleMotorSubsystem.ExampleMotorSubsystemConstants;
import frc.robot.subsystems.exampleMotorSubsystem.ExampleMotorSubsystem;

public class ExampleMotorCmd extends Command {
    private final ExampleMotorSubsystem m_exampleMotorSubsystem;
    private final double power;

    public ExampleMotorCmd(ExampleMotorSubsystem exampleMotorSubsystem, double power) {
        m_exampleMotorSubsystem = exampleMotorSubsystem;
        this.power = power;
        addRequirements(m_exampleMotorSubsystem);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Subsystems/exampleSubsystemCommand/CommandScheduled", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("Subsystems/exampleSubsystemCommand/Speed", power);
        m_exampleMotorSubsystem.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        m_exampleMotorSubsystem.stop();
        Logger.recordOutput("Subsystems/exampleSubsystemCommand/CommandScheduled", false);
    }

    @Override
    public boolean isFinished() {
        /*
         * Because this command lacks a finish state, it's important to make sure it ends some other
         * way, such as using whileTrue (which will make the command end when the button is
         * released) or a raceGroup, which finishes all commands in the group when any command in
         * the group finishes.
         */
        return false;
    }

}
