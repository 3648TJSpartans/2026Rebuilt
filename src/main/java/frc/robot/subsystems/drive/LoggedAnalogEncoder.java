package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.util.TunableNumber;
import java.util.ArrayList;

public class LoggedAnalogEncoder {
  private static final ArrayList<LoggedAnalogEncoder> encoders = new ArrayList<>();

  private final String name;
  private final TunableNumber tunableZero;
  private final int index;
  private AnalogEncoder encoder;
  private final double convFactor;

  public LoggedAnalogEncoder(String name, int index, double convFactor, double zero) {
    tunableZero = new TunableNumber(name + "/zeroOffset", zero);
    this.name = name;
    this.convFactor = convFactor;
    this.index = index;
    encoder = new AnalogEncoder(index, convFactor, tunableZero.get());
    encoders.add(this);
  }

  public void updateZero() {
    encoder.close();
    encoder = new AnalogEncoder(index, convFactor, tunableZero.get());
  }

  public double get() {
    return encoder.get();
  }

  public static void updateZeros() {
    for (LoggedAnalogEncoder encoder : encoders) {
      encoder.updateZero();
    }
  }
}
