package frc.utility;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Field;

public class PositionVector {
  // 218.353069 in
  public static Translation2d getVectorToSpeaker(double x, double y) { // Position of robot relative
    // to origin ()
    double distance;
    double angleRadians;
    Rotation2d angle;
    Translation2d vector;
    double yDistToTarget;
    distance = Math.sqrt((x * x) + ((y - Field.SpeakerYPosMeters) * (y - Field.SpeakerYPosMeters)));

    yDistToTarget = y - Field.SpeakerYPosMeters;
    angleRadians = Math.atan2(yDistToTarget, x);

    angle = new Rotation2d(angleRadians); // angle in radians//

    vector = new Translation2d(distance, angle);
    return vector;
  }
}
