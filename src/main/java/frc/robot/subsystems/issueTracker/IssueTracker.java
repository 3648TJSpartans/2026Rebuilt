package frc.robot.subsystems.issueTracker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IssueTracker extends SubsystemBase {

  private static ArrayList<Requirement> requirements = new ArrayList<Requirement>();

  public IssueTracker() {}

  public static void putRequirements(String name, BooleanSupplier check) {
    requirements.add(new Requirement(name, check));
  }

  public static boolean getRequirement(int index) {
    return requirements.get(index).check.getAsBoolean();
  }

  @Override
  public void periodic() {
    for (Requirement requirement : requirements) {
      Logger.recordOutput("IssueTracker/" + requirement.getName(), requirement.getCheck());
    }
  }

  public static class Requirement {

    protected final String name;
    protected final BooleanSupplier check;

    public Requirement(String name, BooleanSupplier check) {
      this.name = name;
      this.check = check;
    }

    public String getName() {
      return this.name;
    }

    public boolean getCheck() {
      return this.check.getAsBoolean();
    }
  }
}
