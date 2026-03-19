package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public final class LedConstants {

  // Values //
  public static int ledLength = 20;
  public static final int ledPWMID = 1;

  public static int leftBufferStart = 0;
  public static int leftBufferEnd = 8;
  public static int centerBufferStart = 9;
  public static int centerBufferEnd = 17;
  public static int rightBufferStart = 18;
  public static int rightBufferEnd = 19;

  public static int statusCheckOffset = 0;
  // Colors & Patterns //

  // Solid Colors
  public static LEDPattern green = LEDPattern.solid(Color.kGreen);
  public static LEDPattern red = LEDPattern.solid(Color.kRed);
  public static LEDPattern blue = LEDPattern.solid(Color.kBlue);
  public static LEDPattern teal = LEDPattern.solid(Color.kTeal);
  public static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
  public static LEDPattern purple = LEDPattern.solid(Color.kPurple);
  public static LEDPattern white = LEDPattern.solid(Color.kWhite);
  public static LEDPattern noColor = LEDPattern.solid(Color.kBlack);
  public static LEDPattern bluered =
      LEDPattern.steps(Map.of(0, Color.kRed, 0.25, Color.kBlue, 0.5, Color.kRed, 1, Color.kBlue));
  // Gradients
  public static LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  public static LEDPattern purpleGradient =
      LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
  public static LEDPattern elevatorGradient =
      LEDPattern.gradient(
          LEDPattern.GradientType.kDiscontinuous,
          Color.kRed,
          Color.kYellow,
          Color.kGreen,
          Color.kTeal); // Stoplight-esque gradient to use on the elevator

  // Animated Patterns
  public static LEDPattern blinkingBlue = blue.blink(Seconds.of(0.5));
  public static LEDPattern blinkingRed = red.blink(Seconds.of(0.5));
  public static LEDPattern blinkingBlueRed = bluered.blink(Seconds.of(0.5));
}
