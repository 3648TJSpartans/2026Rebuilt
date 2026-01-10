package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.goToCommands.goToConstants.PoseConstants;
import frc.robot.subsystems.absoluteEncoder.AbsEncoder;
import frc.robot.subsystems.absoluteEncoder.AbsEncoderConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.relativeEncoder.RelEncoder;
import frc.robot.subsystems.relativeEncoder.RelEncoderConstants;

/*
 * Command groups can be used to chain multiple commands together. For example, if you want to move
 * one motor, then drive somewhere, then move another motor (as shown in this example), you can use
 * command groups to do so.
 * 
 * This command group is sequential, which means the first command finishes, and then the next one
 * starts. Other command groups, like parallel groups, in which two commands are both executed
 * simultaneously, can be used as well.
 */

public class ExampleSequentialCmd extends SequentialCommandGroup {

    private final RelEncoder m_relEncoder;
    private final AbsEncoder m_absEncoder;
    private final Drive m_drive;
    private final Command m_relCommand;
    private final Command m_absCommand;

    public ExampleSequentialCmd(Drive drive, AbsEncoder absEncoder, RelEncoder relEncoder) {

        m_drive = drive;
        m_relEncoder = relEncoder;
        m_absEncoder = absEncoder;

        m_relCommand = AutoBuildingBlocks.relCmd(m_relEncoder, RelEncoderConstants.setpoint1);
        m_absCommand = AutoBuildingBlocks.absCmd(m_absEncoder, AbsEncoderConstants.setpoint1);

        addCommands(new SequentialCommandGroup(m_relCommand, m_absCommand));
    }

}
