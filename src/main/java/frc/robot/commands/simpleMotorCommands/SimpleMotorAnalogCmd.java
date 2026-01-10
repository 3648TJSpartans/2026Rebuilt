package frc.robot.commands.simpleMotorCommands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.simpleMotor.SimpleMotor;

public class SimpleMotorAnalogCmd extends Command {
    private final SimpleMotor m_simpleMotor;
    private final Supplier<Double> speed;

    public SimpleMotorAnalogCmd(SimpleMotor simpleMotor, Supplier<Double> speed) {
        m_simpleMotor = simpleMotor;
        this.speed = speed;
        addRequirements(m_simpleMotor);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("simpleMotor/AnalogCommand/Scheduled", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("simpleMotor/AnalogCommand/Speed", speed.get());
        m_simpleMotor.setSpeed(speed.get());
    }

    @Override
    public void end(boolean interrupted) {
        m_simpleMotor.stop();
        Logger.recordOutput("simpleMotor/AnalogCommand/Scheduled", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}