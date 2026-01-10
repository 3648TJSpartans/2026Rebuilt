package frc.robot.util;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class Diagnostics {

    private static ArrayList<Requirement> requirements;

    public static void putRequirements(String name, BooleanSupplier check) {
        requirements.add(new Requirement(name, check));
    }

    public static boolean getRequirement(int index) {
        return requirements.get(index).check.getAsBoolean();
    }

    public static class Requirement {

        protected final String name;
        protected final BooleanSupplier check;

        public Requirement(String name, BooleanSupplier check) {
            this.name = name;
            this.check = check;
        }
    }

}
