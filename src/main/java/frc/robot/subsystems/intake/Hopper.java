package frc.robot.subsystems.intake;

import frc.robot.Constants;
import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends RelEncoderSparkMax {
  private boolean overrideJam;
  private RelEncoderSparkMax followMotor;

  public Hopper() {
    super(IntakeConstants.hopperMotorConfig);
    followMotor = new RelEncoderSparkMax(IntakeConstants.followHopperMotorConfig);
    overrideJam = false;
  }

  @AutoLogOutput(key = "Subsystems/Intake/Hopper/currentToSpeed")
  public double getCurrentToSpeed() {
    return Math.abs(getSpeed()) > IntakeConstants.speedThreshold.get()
            && Math.abs(getCurrent()) > IntakeConstants.currentThreshold.get()
        ? Math.abs(getCurrent() / getSpeed())
        : 0.0;
  }

  @AutoLogOutput(key = "Subsystems/Intake/Hopper/jammed")
  public boolean jammed() {
    return getCurrentToSpeed() > IntakeConstants.jamThreshold.get();
  }

  public void overrideJam(boolean override) {
    overrideJam = override;
  }

  @AutoLogOutput(key = "Subsystems/Intake/Hopper/overrideJam")
  public boolean getOverrideJam() {
    return overrideJam;
  }

  @Override
  public void stop() {
    super.stop();
    followMotor.stop();
  }

  @Override
  public void setPower(double power) {
    super.setPower(power);
    followMotor.setPower(power);
  }

  public void run() {
    if (overrideJam && IntakeConstants.unjamEnabled.get()) {
      setPower(IntakeConstants.hopperUnjamPower.get());
      Logger.recordOutput("Subsystems/Intake/Hopper/outtake", true);
      return;
    }
    Logger.recordOutput("Subsystems/Intake/Hopper/outtake", false);
    if (IntakeConstants.runHopperPID.get()) {
      setSpeed(IntakeConstants.hopperSpeed.get());
      return;
    }
    setPower(IntakeConstants.hopperPower.get());
  }

  @Override
  public Status getStatus() {
    return Constants.leastCommonStatus(super.getStatus(), followMotor.getStatus());
  }
}
