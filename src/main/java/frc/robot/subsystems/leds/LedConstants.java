package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public final class LedConstants {

  // Values //
  public static int ledLength = 49;
  public static final int ledPWMID = 1;
  public static final int buffer1StartLeft = 0;
  public static final int buffer1EndLeft = 14;
  public static final int buffer2StartLeft = 15;
  public static final int buffer2EndLeft = 29;
  public static final int buffer3StartLeft = 30;
  public static final int buffer3EndLeft = 44;
  public static final int buffer4StartLeft = 45;
  public static final int buffer4EndLeft = 59;
  // right side
  public static final int buffer1StartRight = 60;
  public static final int buffer1EndRight = 74;
  public static final int buffer2StartRight = 75;
  public static final int buffer2EndRight = 89;
  public static final int buffer3StartRight = 90;
  public static final int buffer3EndRight = 104;
  public static final int buffer4StartRight = 105;
  public static final int buffer4EndRight = 119;
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
  // public static LEDPattern rainbow = LEDPattern.rainbow(255, 128);
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
