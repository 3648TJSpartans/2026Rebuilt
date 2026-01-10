package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TuningUpdater extends SubsystemBase {
    /** If true, TunableNumbers are on, else, they can't be changed. */
    public static boolean TUNING_MODE = true;
    public static final String TABLE_KEY = "TunableNumbers";
    public static final String dashBoardKey = "TuningOn";

    private final String key = TABLE_KEY + "/" + dashBoardKey;

    public TuningUpdater() {
        SmartDashboard.putBoolean(key,
                SmartDashboard.getBoolean(key, Constants.DEFAULT_TUNING_MODE));
    }

    @Override
    public void periodic() {
        TUNING_MODE = SmartDashboard.getBoolean(key, Constants.DEFAULT_TUNING_MODE);
        Logger.recordOutput(key, TUNING_MODE);
    }
}
