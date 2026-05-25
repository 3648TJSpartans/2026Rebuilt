package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.revrobotics.spark.SparkMax;
import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

public class MappedSparkMax extends SparkMax {
  private static final Map<Integer, String> sparkMaxMap = new TreeMap<>();

  public MappedSparkMax(int deviceID, MotorType type, String mapName) {
    super(deviceID, type);
    Logger.recordOutput("Utils/MappedSparkMax/map/" + deviceID, mapName);
    sparkMaxMap.put(deviceID, mapName);
  }

  public static void logSparkMaxes() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.enable(SerializationFeature.INDENT_OUTPUT);
    try {
      mapper.writeValue(new File("/U/logs/MotorMap.json"), sparkMaxMap);
      Logger.recordOutput("Utils/MappedSparkMax/loggedJson", true);
    } catch (IOException e) {
      Logger.recordOutput("Utils/MappedSparkMax/loggedJson", false);
    }
  }
}
