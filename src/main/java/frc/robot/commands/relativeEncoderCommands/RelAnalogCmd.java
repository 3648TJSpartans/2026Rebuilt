package frc.robot.commands.relativeEncoderCommands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.relativeEncoder.RelEncoder;

public class RelAnalogCmd extends Command {
    private final RelEncoder m_relEncoder;
    private final Supplier<Double> speed;

    public RelAnalogCmd(RelEncoder relEncoder, Supplier<Double> speed) {
        m_relEncoder = relEncoder;
        this.speed = speed;
        addRequirements(m_relEncoder);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("relEncoder/AnalogCommand/Scheduled", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("relEncoder/AnalogCommand/Speed", speed.get());
        m_relEncoder.setSpeed(speed.get());
    }

    @Override
    public void end(boolean interrupted) {
        m_relEncoder.stop();
        Logger.recordOutput("relEncoder/AnalogCommand/Scheduled", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}