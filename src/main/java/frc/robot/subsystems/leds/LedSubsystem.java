package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}
