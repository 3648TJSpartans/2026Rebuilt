package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Status;
import frc.robot.util.statusableUtils.Statusable;
import org.littletonrobotics.junction.Logger;

public class SmartController extends CommandXboxController implements Statusable {

  private Status m_status;
  private final String m_name;

  private SmartButton[] buttonArray;
  private SmartPOV[] povArray;
  private SmartAxis[] axisArray;

  private class SmartButton {

    public boolean allocated;
    public int value;
    private String name;

    public SmartButton(String name, int value) {
      this.name = name;
      this.value = value;
      this.allocated = false;
    }

    @Override
    public String toString() {
      return name + " button";
    }
  }

  private class SmartPOV {

    public boolean allocated;
    public int angle;
    private String name;

    public SmartPOV(String name, int angle) {
      this.name = name;
      this.angle = angle;
      this.allocated = false;
    }

    @Override
    public String toString() {
      return "POV " + name;
    }
  }

  private class SmartAxis {

    public boolean allocated;
    public int value;
    private String name;

    public SmartAxis(String name, int value) {
      this.name = name;
      this.value = value;
      this.allocated = false;
    }

    @Override
    public String toString() {
      return name + " axis";
    }
  }

  public SmartController(String name, int port) {
    super(port);
    m_name = name;
    m_status = Status.OK;
    buttonArray =
        new SmartButton[] {
          new SmartButton("Placeholder", 0),
          new SmartButton("A", 1),
          new SmartButton("B", 2),
          new SmartButton("X", 3),
          new SmartButton("Y", 4),
          new SmartButton("leftBumper", 5),
          new SmartButton("rightBumper", 6),
          new SmartButton("back", 7),
          new SmartButton("start", 8),
          new SmartButton("leftStick", 9),
          new SmartButton("rightStick", 10)
        };
    povArray =
        new SmartPOV[] {
          new SmartPOV("Up", 0),
          new SmartPOV("Right", 90),
          new SmartPOV("Down", 180),
          new SmartPOV("Left", 270)
        };
    axisArray =
        new SmartAxis[] {
          new SmartAxis("leftX", 0),
          new SmartAxis("leftY", 1),
          new SmartAxis("leftTrigger", 2),
          new SmartAxis("rightTrigger", 3),
          new SmartAxis("rightX", 4),
          new SmartAxis("rightY", 5)
        };
    Logger.recordOutput("Utils/SmartController/" + m_name + "/rumbling?", false);
  }

  public void rumble(double value) {
    Logger.recordOutput(
        "Utils/SmartController/" + m_name + "/rumbling?", value != 0 ? true : false);
    getHID().setRumble(RumbleType.kBothRumble, value);
  }

  public void rumble() {
    Logger.recordOutput("Utils/SmartController/" + m_name + "/rumbling?", true);
    getHID().setRumble(RumbleType.kBothRumble, 1);
  }

  public void stopRumble() {
    Logger.recordOutput("Utils/SmartController/" + m_name + "/rumbling?", false);
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
    return getButton(buttonArray[XboxController.Button.kA.value]);
  }

  @Override
  public Trigger b() {
    return getButton(buttonArray[XboxController.Button.kB.value]);
  }

  @Override
  public Trigger x() {
    return getButton(buttonArray[XboxController.Button.kX.value]);
  }

  @Override
  public Trigger y() {
    return getButton(buttonArray[XboxController.Button.kY.value]);
  }

  @Override
  public Trigger leftBumper() {
    return getButton(buttonArray[XboxController.Button.kLeftBumper.value]);
  }

  @Override
  public Trigger rightBumper() {
    return getButton(buttonArray[XboxController.Button.kRightBumper.value]);
  }

  @Override
  public Trigger back() {
    return getButton(buttonArray[XboxController.Button.kBack.value]);
  }

  @Override
  public Trigger start() {
    return getButton(buttonArray[XboxController.Button.kStart.value]);
  }

  @Override
  public Trigger leftStick() {
    return getButton(buttonArray[XboxController.Button.kLeftStick.value]);
  }

  @Override
  public Trigger rightStick() {
    return getButton(buttonArray[XboxController.Button.kRightStick.value]);
  }

  @Override
  public Trigger povUp() {
    return getPOV(povArray[0]);
  }

  @Override
  public Trigger povRight() {
    return getPOV(povArray[1]);
  }

  @Override
  public Trigger povDown() {
    return getPOV(povArray[2]);
  }

  @Override
  public Trigger povLeft() {
    return getPOV(povArray[3]);
  }

  @Override
  public Trigger leftTrigger() {
    if (axisArray[XboxController.Axis.kLeftTrigger.value].allocated) {
      m_status = Status.WARNING;
      Logger.recordOutput(
          "Debug/" + m_name + "/AxisAllocation/",
          axisArray[XboxController.Axis.kLeftTrigger.value] + " is allocated multiple times.");
    }
    return super.leftTrigger();
  }

  @Override
  public Trigger rightTrigger() {
    if (axisArray[XboxController.Axis.kRightTrigger.value].allocated) {
      m_status = Status.WARNING;
      Logger.recordOutput(
          "Debug/" + m_name + "/AxisAllocation/",
          axisArray[XboxController.Axis.kRightTrigger.value] + " is allocated multiple times.");
    }
    return super.rightTrigger();
  }

  @Override
  public double getLeftX() {
    return getAxis(axisArray[XboxController.Axis.kLeftX.value]);
  }

  @Override
  public double getLeftY() {
    return getAxis(axisArray[XboxController.Axis.kLeftY.value]);
  }

  @Override
  public double getRightX() {
    return getAxis(axisArray[XboxController.Axis.kRightX.value]);
  }

  @Override
  public double getRightY() {
    return getAxis(axisArray[XboxController.Axis.kRightY.value]);
  }

  @Override
  public double getLeftTriggerAxis() {
    return getAxis(axisArray[XboxController.Axis.kLeftTrigger.value]);
  }

  @Override
  public double getRightTriggerAxis() {
    return getAxis(axisArray[XboxController.Axis.kRightTrigger.value]);
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
