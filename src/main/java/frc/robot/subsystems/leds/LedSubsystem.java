package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BuildConstants;
import frc.robot.RobotContainer.Status;

public class LedSubsystem extends SubsystemBase {

  public final AddressableLEDBuffer ledBuffer;

  public final AddressableLEDBufferView exampleView;

  private final AddressableLED led;

  public LedSubsystem() {
    led = new AddressableLED(LedConstants.ledPWMID);
    ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
    exampleView = new AddressableLEDBufferView(ledBuffer, 0, 10);

    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void turnLedsOff() {

    LedConstants.noColor.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  // left or right
  public void setGlobalPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void setPattern(AddressableLEDBufferView buffer, LEDPattern pattern) {
    pattern.applyTo(buffer);
  }

  public void setSingleLed(int r, int g, int b, int i) {
    ledBuffer.setRGB(i, r, g, b);
    led.setData(ledBuffer);
  }

  public void setSingleLedByStatus(int i, Status status) {
    // TODO Auto-generated method stub
    switch (status) {
      case OK:
        setSingleLed(0, 255, 0, i);
        break;
      case WARNING:
        setSingleLed(255, 175, 0, i);
        break;
      case ERROR:
        setSingleLed(255, 0, 0, i);
        break;
      default:
        setSingleLed(133, 50, 168, i);
        break;
    }
  }
}

/*
 * public void case
 * States = OK setSingleLed(0, 2255 ,0);
 */