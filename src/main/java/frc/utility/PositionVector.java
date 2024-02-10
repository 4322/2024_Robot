package frc.utility;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class PositionVector {
  // 218.353069 in
  public static Translation2d getVectorToSpeaker(
      double x, double y) { // Position of robot relative to origin ()
    double distance;
    double angleRadians;
    Rotation2d angle;
    Translation2d vector;
    double xDistToTarget;
    double yDistToTarget;

    distance =
        Math.sqrt(
            ((x - FieldConstants.xSpeakerPosM) * (x - FieldConstants.xSpeakerPosM))
                + ((y - FieldConstants.ySpeakerPosM) * (y - FieldConstants.ySpeakerPosM)));

    xDistToTarget = Math.abs(x - FieldConstants.xSpeakerPosM);
    yDistToTarget = Math.abs(y - FieldConstants.ySpeakerPosM);

    angleRadians = Math.atan2(yDistToTarget, xDistToTarget);

    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      angleRadians = -angleRadians; // Flip by pi radian if red
      // Assumes that 0 is alwyas facing away from speaker
    }

    angle = new Rotation2d(angleRadians); // angle in radians
    vector = new Translation2d(distance, angle);
    return vector;
  }
}
