package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public final class LedConstants {

  // Values //
  public static int ledLength = 25;
  public static final int ledPWMID = 1;

  public static int leftBufferStart = 0;
  public static int leftBufferEnd = 8;
  public static int centerBufferStart = leftBufferStart + leftBufferEnd;
  public static int centerBufferEnd = 9;
  public static int rightBufferStart = centerBufferStart + centerBufferEnd;
  public static int rightBufferEnd = 8;

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
  public static LEDPattern breathingGreen = green.breathe(Seconds.of(2));
  public static LEDPattern blinkingBlue = blue.breathe(Seconds.of(1.5));
  public static LEDPattern blinkingteal = teal.breathe(Seconds.of(1.5));
}
