package frc.robot.util;

import com.revrobotics.spark.SparkMax;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

public class MappedSparkMax extends SparkMax {
  private static final Map<Integer, String> sparkMaxMap = new TreeMap<>();

  public MappedSparkMax(int deviceID, MotorType type, String mapName) {
    super(deviceID, type);
    Logger.recordOutput("Utils/MappedSparkMax/" + deviceID, mapName);
    sparkMaxMap.put(deviceID, mapName);
  }

  public static void logSparkMaxes() {
    // for (Map.Entry<String, Integer> entry : sparkMaxMap.entrySet()) {
    //   Logger.recordOutput("Utils/MappedSparkMax/" + entry.getKey(), entry.getValue());
    // }
  }
}
