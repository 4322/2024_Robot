package frc.utility;

import edu.wpi.first.math.geometry.Translation2d;

public class SnapshotTranslation2D {
  private Translation2d translation2d;
  private double time;

  public SnapshotTranslation2D(Translation2d t2d, double time) {
    this.translation2d = t2d;
    this.time = time;
  }

  public Translation2d getTranslation2d() {
    return translation2d;
  }

  public double getTime() {
    return time;
  }
}
