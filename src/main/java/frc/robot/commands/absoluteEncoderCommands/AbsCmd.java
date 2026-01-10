package frc.robot.commands.absoluteEncoderCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.absoluteEncoder.AbsEncoder;
import frc.robot.subsystems.absoluteEncoder.AbsEncoderConstants;

public class AbsCmd extends Command {
    private final AbsEncoder m_absEncoder;
    private final double position;

    public AbsCmd(AbsEncoder absEncoder, double position) {
        m_absEncoder = absEncoder;
        this.position = position;
        addRequirements(m_absEncoder);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("absEncoder/CommandScheduled", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("absEncoder/setPosition", position);
        Logger.recordOutput("absEncoder/position", m_absEncoder.getPosition());
        m_absEncoder.setTo(position);
    }

    @Override
    public void end(boolean interrupted) {
        m_absEncoder.stop();
        Logger.recordOutput("absEncoder/CommandScheduled", false);
    }

    @Override
    public boolean isFinished() {
        /*
         * A finish state is essential to commands that are used in command groups
         * (see command group file for more details). The code below finishes the
         * command if it's within a certain constant margin, the setpointTolerance.
         */
        return ((position - AbsEncoderConstants.setpointTolerance) < m_absEncoder.getPosition()
                &&
                m_absEncoder.getPosition() < (position + AbsEncoderConstants.setpointTolerance));
    }

}