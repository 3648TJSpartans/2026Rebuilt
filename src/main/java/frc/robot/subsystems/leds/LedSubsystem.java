package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  public final AddressableLEDBuffer ledBuffer;

  public final AddressableLEDBufferView exampleView;
  public final AddressableLEDBufferView leftBuffer; // Left side of the intake
  public final AddressableLEDBufferView centerBuffer1; // Left side of top of intake
  public final AddressableLEDBufferView centerBuffer2; // Right side of top of intake
  public final AddressableLEDBufferView centerBuffer3;
  public final AddressableLEDBufferView rightBuffer; // Right side of the intake

  private final AddressableLED led;

  public LedSubsystem() {
    led = new AddressableLED(LedConstants.ledPWMID);
    ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
    exampleView = new AddressableLEDBufferView(ledBuffer, 0, 10);
    leftBuffer =
        new AddressableLEDBufferView(
            ledBuffer, LedConstants.leftBufferStart, LedConstants.leftBufferEnd);
    centerBuffer1 =
        new AddressableLEDBufferView(
            ledBuffer, LedConstants.centerBuffer1Start, LedConstants.centerBuffer1End);
    centerBuffer2 =
        new AddressableLEDBufferView(
            ledBuffer, LedConstants.centerBuffer2Start, LedConstants.centerBuffer2End);
    centerBuffer3 =
        new AddressableLEDBufferView(
            ledBuffer, LedConstants.centerBuffer3Start, LedConstants.centerBuffer3End);
    rightBuffer =
        new AddressableLEDBufferView(
            ledBuffer, LedConstants.rightBufferStart, LedConstants.rightBufferEnd);

    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void turnLedsOff() {
    LedConstants.noColor.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  /**
   * Sets the entire LED strip to a given pattern, overwriting any previous data on the strip.
   *
   * @param pattern - The pattern to set the LEDs to.
   */
  public void setGlobalPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  /**
   * Sets a given pattern to a given sub-buffer.
   *
   * @param buffer - The buffer view to set the pattern on.
   * @param pattern - The pattern to set the buffer view to.
   */
  public void setPattern(AddressableLEDBufferView buffer, LEDPattern pattern) {
    pattern.applyTo(buffer);
  }

  /**
   * Sets a single LED to a given RGB value.
   *
   * @param r - The red value (0-255).
   * @param g - The green value (0-255).
   * @param b - The blue value (0-255).
   * @param i - The index of the LED to set.
   */
  public void setSingleLed(int r, int g, int b, int i) {
    ledBuffer.setRGB(i, r, g, b);
    led.setData(ledBuffer);
  }
}
