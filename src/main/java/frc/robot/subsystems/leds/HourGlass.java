package frc.robot.subsystems.leds;

public class HourGlass {
  private double m_topVolume;
  private double m_bottomVolume;
  private static final double RATE = 0.01;

  // public static void main(String[] args) {
  //   HourGlass hourGlass = new HourGlass();
  //   for (int i = 0; i < 100; i++) {
  //     hourGlass.updateHourGlass(3 * Math.PI / 4);
  //     System.out.println(hourGlass);
  //     try {
  //       Thread.sleep(100);
  //     } catch (InterruptedException e) {
  //       e.printStackTrace();
  //     }
  //   }
  // }

  public HourGlass(double topVolume, double bottomVolume) {
    this.m_topVolume = topVolume;
    this.m_bottomVolume = bottomVolume;
  }

  public HourGlass() {
    this(0.5, 0.5);
  }

  public void updateHourGlass(double angle) {
    double rate = RATE * Math.cos(angle);
    if (rate > 0) {
      if (m_topVolume - rate < 0) {
        return;
      }
    }
    if (rate < 0) {
      if (m_bottomVolume + rate < 0) {
        return;
      }
    }
    m_topVolume -= rate;
    m_bottomVolume += rate;
  }

  public double getTopVolume() {
    return m_topVolume;
  }

  public double getBottomVolume() {
    return m_bottomVolume;
  }

  public String toString() {
    return "HourGlass{topVolume=" + m_topVolume + ", bottomVolume=" + m_bottomVolume + "}";
  }
}
