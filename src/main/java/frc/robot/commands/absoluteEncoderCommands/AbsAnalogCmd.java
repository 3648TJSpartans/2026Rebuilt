package frc.robot.commands.absoluteEncoderCommands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.absoluteEncoder.AbsEncoder;

public class AbsAnalogCmd extends Command {
    private final AbsEncoder m_absEncoder;
    private final Supplier<Double> speed;

    public AbsAnalogCmd(AbsEncoder absEncoder, Supplier<Double> speed) {
        m_absEncoder = absEncoder;
        this.speed = speed;
        addRequirements(m_absEncoder);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("absEncoder/AnalogCommand/Scheduled", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("absEncoder/AnalogCommand/Speed", speed.get());
        m_absEncoder.setSpeed(speed.get());
    }

    @Override
    public void end(boolean interrupted) {
        m_absEncoder.stop();
        Logger.recordOutput("absEncoder/AnalogCommand/Scheduled", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}