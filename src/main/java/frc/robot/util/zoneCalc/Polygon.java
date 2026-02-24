package frc.robot.util.zoneCalc;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Polygon {
  private final Translation2d[] vertices;
  private final String name;

  public Polygon(String name, Translation2d... vertices) {
    this.name = name;
    this.vertices = vertices;
    ArrayList<Translation2d> out = new ArrayList<>(Arrays.asList(vertices));
    out.add(vertices[0]);
    Logger.recordOutput("Utils/ZoneCalc/" + name, out.toArray(new Translation2d[0]));
  }

  public Translation2d[] getCorners() {
    return vertices;
  }

  /** Checks if a single point is inside this polygon. */
  public boolean contains(Translation2d point) {
    boolean inside = false;
    int n = vertices.length;
    for (int i = 0, j = n - 1; i < n; j = i++) {
      if (((vertices[i].getY() > point.getY()) != (vertices[j].getY() > point.getY()))
          && (point.getX()
              < (vertices[j].getX() - vertices[i].getX())
                      * (point.getY() - vertices[i].getY())
                      / (vertices[j].getY() - vertices[i].getY())
                  + vertices[i].getX())) {
        inside = !inside;
      }
    }
    return inside;
  }

  /**
   * Checks if ANY corner of the 'other' polygon is within this polygon. Use 'allMatch' instead of
   * 'anyMatch' if you want to check for full containment.
   */
  public boolean contains(Polygon other) {
    Translation2d[] v1 = this.getCorners();
    Translation2d[] v2 = other.getCorners();

    // Check if any corner of 'other' is inside 'this'
    for (Translation2d p : v2) {
      if (this.contains(p)) return true;
    }

    // Check if any corner of 'this' is inside 'other'
    for (Translation2d p : v1) {
      if (other.contains(p)) return true;
    }

    // Check every edge of 'this' against every edge of 'other'
    for (int i = 0; i < v1.length; i++) {
      Translation2d a = v1[i];
      Translation2d b = v1[(i + 1) % v1.length];

      for (int j = 0; j < v2.length; j++) {
        Translation2d c = v2[j];
        Translation2d d = v2[(j + 1) % v2.length];

        if (doEdgesIntersect(a, b, c, d)) {
          return true;
        }
      }
    }

    return false;
  }

  /** Returns true only if the 'other' polygon is entirely inside this polygon. */
  public boolean fullyContains(Polygon other) {
    Translation2d[] thisCorners = this.getCorners();
    Translation2d[] otherCorners = other.getCorners();

    // 1. Every corner of the other polygon MUST be inside this polygon
    for (Translation2d p : otherCorners) {
      if (!this.contains(p)) {
        return false;
      }
    }

    // 2. No edges can intersect.
    // If they do, the other polygon is "poking out" or crossing a boundary.
    for (int i = 0; i < thisCorners.length; i++) {
      Translation2d a = thisCorners[i];
      Translation2d b = thisCorners[(i + 1) % thisCorners.length];

      for (int j = 0; j < otherCorners.length; j++) {
        Translation2d c = otherCorners[j];
        Translation2d d = otherCorners[(j + 1) % otherCorners.length];

        if (doEdgesIntersect(a, b, c, d)) {
          return false;
        }
      }
    }

    return true;
  }

  /** Helper to see if line segment ab intersects line segment cd. */
  private boolean doEdgesIntersect(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    double denominator =
        (b.getX() - a.getX()) * (d.getY() - c.getY())
            - (b.getY() - a.getY()) * (d.getX() - c.getX());

    if (denominator == 0) return false; // Parallel lines

    double t =
        ((c.getX() - a.getX()) * (d.getY() - c.getY())
                - (c.getY() - a.getY()) * (d.getX() - c.getX()))
            / denominator;
    double u =
        ((c.getX() - a.getX()) * (b.getY() - a.getY())
                - (c.getY() - a.getY()) * (b.getX() - a.getX()))
            / denominator;

    return (t >= 0 && t <= 1) && (u >= 0 && u <= 1);
  }

  public String getName() {
    return name;
  }
}
