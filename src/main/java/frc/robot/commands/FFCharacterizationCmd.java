package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.TunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FFCharacterizationCmd extends Command {
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final TunableNumber FF_RAMP_RATE =
      new TunableNumber("Utils/FFCharacterizer/rampRate", 0.5); // Volts/Sec

  public static Command characterizeSystem(
      Subsystem system,
      Consumer<Double> runCharacterization,
      Supplier<Double> getFFCharacterizationVelocity) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();
    double rampRate = FF_RAMP_RATE.get();
    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Set speed to 0
        Commands.run(
                () -> {
                  runCharacterization.accept(0.0);
                },
                system)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * rampRate;
                  runCharacterization.accept(voltage);
                  velocitySamples.add(getFFCharacterizationVelocity.get());
                  voltageSamples.add(voltage);
                },
                system)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                  Logger.recordOutput("Utils/FFCharacterizer/kSResult", formatter.format(kS));
                  Logger.recordOutput("Utils/FFCharacterizer/kVResult", formatter.format(kV));
                }));
  }
}
