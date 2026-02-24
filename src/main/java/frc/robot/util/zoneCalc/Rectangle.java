package frc.robot.util.zoneCalc;

import edu.wpi.first.math.geometry.Translation2d;

public class Rectangle extends Polygon {
  /**
   * Creates an axis-aligned rectangle from two opposite corners.
   *
   * @param bottomCorner Usually the min X and min Y
   * @param topCorner Usually the max X and max Y
   */
  public Rectangle(String name, Translation2d bottomCorner, Translation2d topCorner) {
    super(name, generateVertices(bottomCorner, topCorner));
  }

  private static Translation2d[] generateVertices(Translation2d p1, Translation2d p2) {
    // Calculate the bounds to ensure it works even if the points are passed out of order
    double minX = Math.min(p1.getX(), p2.getX());
    double maxX = Math.max(p1.getX(), p2.getX());
    double minY = Math.min(p1.getY(), p2.getY());
    double maxY = Math.max(p1.getY(), p2.getY());

    // Define the 4 corners in order (Clockwise or Counter-Clockwise)
    return new Translation2d[] {
      new Translation2d(minX, minY), // Bottom-Left
      new Translation2d(minX, maxY), // Top-Left
      new Translation2d(maxX, maxY), // Top-Right
      new Translation2d(maxX, minY) // Bottom-Right
    };
  }
}
