package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Status;
import frc.robot.util.statusableUtils.Statusable;
import org.littletonrobotics.junction.Logger;

public class SmartController extends CommandXboxController implements Statusable {

  private Status m_status;
  private final String m_name;

  public enum SmartButton {
    /** A button. */
    kA(1),
    /** B button. */
    kB(2),
    /** X button. */
    kX(3),
    /** Y button. */
    kY(4),
    /** Left bumper button. */
    kLeftBumper(5),
    /** Right bumper button. */
    kRightBumper(6),
    /** Back button. */
    kBack(7),
    /** Start button. */
    kStart(8),
    /** Left stick button. */
    kLeftStick(9),
    /** Right stick button. */
    kRightStick(10),
    /** Left Stick X */
    kLeftX(11),
    /** Left Stick Y */
    kLeftY(12),
    /** Right Stick X */
    kRightX(13),
    /** Right Stick Y */
    kRightY(14);

    /** Button value. */
    public final int value;

    public boolean allocated;

    SmartButton(int value) {
      this.value = value;
      this.allocated = false;
    }

    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  public enum SmartPOV {
    kPOVUp(0),
    kPOVRight(90),
    kPOVDown(180),
    kPOVLeft(270);

    public int angle;
    public boolean allocated;

    SmartPOV(int angle) {
      this.angle = angle;
      this.allocated = false;
    }

    @Override
    public String toString() {
      return this.name().substring(1); // removing leading k
    }
  }

  public enum SmartAxis {
    /** Left X axis. */
    kLeftX(0),
    /** Right X axis. */
    kRightX(4),
    /** Left Y axis. */
    kLeftY(1),
    /** Right Y axis. */
    kRightY(5),
    /** Left trigger. */
    kLeftTrigger(2),
    /** Right trigger. */
    kRightTrigger(3);

    /** Axis value. */
    public final int value;

    public boolean allocated;

    SmartAxis(int value) {
      this.value = value;
      this.allocated = false;
    }

    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  public SmartController(String name, int port) {
    super(port);
    m_name = name;
    m_status = Status.OK;
    Logger.recordOutput(m_name + "/rumbling?", false);
  }

  public void rumble(double value) {
    Logger.recordOutput(m_name + "/rumbling?", value != 0 ? true : false);
    getHID().setRumble(RumbleType.kBothRumble, value);
  }

  public void rumble() {
    Logger.recordOutput(m_name + "/rumbling?", true);
    getHID().setRumble(RumbleType.kBothRumble, 1);
  }

  public void stopRumble() {
    Logger.recordOutput(m_name + "/rumbling?", false);
    getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  private Trigger getButton(SmartButton button) {
    if (button.allocated) {
      m_status = Status.WARNING;
      Logger.recordOutput(
          "Debug/" + m_name + "/ButtonAllocation/" + button,
          button + " is allocated multiple times.");
    }
    System.out.println(m_name + " allocated at " + button);

    button.allocated = true;
    return super.button(button.value);
  }

  private Trigger getPOV(SmartPOV pov) {
    if (pov.allocated) {
      m_status = Status.WARNING;
      Logger.recordOutput(
          "Debug/" + m_name + "/PovAllocation/" + pov, pov + " is allocated multiple times.");
    }
    pov.allocated = true;
    return super.pov(pov.angle);
  }

  private double getAxis(SmartAxis axis) {
    if (axis.allocated) {
      m_status = Status.WARNING;
      Logger.recordOutput(
          "Debug/" + m_name + "/AxisAllocation/" + axis, axis + " is allocated multiple times.");
    }
    axis.allocated = true;
    return super.getRawAxis(axis.value);
  }

  @Override
  public Trigger a() {
    return getButton(SmartButton.kA);
  }

  @Override
  public Trigger b() {
    return getButton(SmartButton.kB);
  }

  @Override
  public Trigger x() {
    return getButton(SmartButton.kX);
  }

  @Override
  public Trigger y() {
    return getButton(SmartButton.kY);
  }

  @Override
  public Trigger leftBumper() {
    return getButton(SmartButton.kLeftBumper);
  }

  @Override
  public Trigger rightBumper() {
    return getButton(SmartButton.kRightBumper);
  }

  @Override
  public Trigger back() {
    return getButton(SmartButton.kBack);
  }

  @Override
  public Trigger start() {
    return getButton(SmartButton.kStart);
  }

  @Override
  public Trigger leftStick() {
    return getButton(SmartButton.kLeftStick);
  }

  @Override
  public Trigger rightStick() {
    return getButton(SmartButton.kRightStick);
  }

  @Override
  public Trigger povUp() {
    return getPOV(SmartPOV.kPOVUp);
  }

  @Override
  public Trigger povRight() {
    return getPOV(SmartPOV.kPOVRight);
  }

  @Override
  public Trigger povDown() {
    return getPOV(SmartPOV.kPOVDown);
  }

  @Override
  public Trigger povLeft() {
    return getPOV(SmartPOV.kPOVLeft);
  }

  @Override
  public Trigger leftTrigger() {
    if (SmartAxis.kLeftTrigger.allocated) {
      m_status = Status.WARNING;
      Logger.recordOutput(
          "Debug/" + m_name + "/AxisAllocation/",
          SmartAxis.kLeftTrigger.allocated + " is allocated multiple times.");
    }
    return super.leftTrigger();
  }

  @Override
  public Trigger rightTrigger() {
    if (SmartAxis.kRightTrigger.allocated) {
      m_status = Status.WARNING;
      Logger.recordOutput(
          "Debug/" + m_name + "/AxisAllocation/",
          SmartAxis.kRightTrigger.allocated + " is allocated multiple times.");
    }
    return super.leftTrigger();
  }

  @Override
  public double getLeftX() {
    return getAxis(SmartAxis.kLeftX);
  }

  @Override
  public double getLeftY() {
    return getAxis(SmartAxis.kLeftY);
  }

  @Override
  public double getRightX() {
    return getAxis(SmartAxis.kRightX);
  }

  @Override
  public double getRightY() {
    return getAxis(SmartAxis.kRightY);
  }

  @Override
  public double getLeftTriggerAxis() {
    return getAxis(SmartAxis.kLeftTrigger);
  }

  @Override
  public double getRightTriggerAxis() {
    return getAxis(SmartAxis.kRightTrigger);
  }

  @Override
  public Status getStatus() {
    if (!isConnected()) {
      Logger.recordOutput("Debug/" + m_name + "/Connection/", "Not connected.");
      return Status.ERROR;
    }
    return m_status;
  }

  @Override
  public String getName() {
    return m_name;
  }
}
