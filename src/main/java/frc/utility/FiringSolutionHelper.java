package frc.utility;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class FiringSolutionHelper {
  // 218.353069 inches
  private static double distance;
  private static double angleRadians;
  private static Rotation2d angle;
  private static Translation2d vector;
  private static double xDistToTarget;
  private static double yDistToTarget;

  public static Translation2d getVectorToSpeaker(
      double x, double y) { // Position of robot relative to origin ()
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

  public static double getMag(double x, double y) {
    return Math.sqrt(
        ((x - FieldConstants.xSpeakerPosM) * (x - FieldConstants.xSpeakerPosM))
            + ((y - FieldConstants.ySpeakerPosM) * (y - FieldConstants.ySpeakerPosM)));
  }

  public static Rotation2d getAngle(double x, double y) {
    // negative on red
    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      return new Rotation2d(
        -Math.atan2(
            Math.abs(y - FieldConstants.ySpeakerPosM), Math.abs(x - FieldConstants.xSpeakerPosM)));
    } else {
      return new Rotation2d(
          Math.atan2(
              Math.abs(y - FieldConstants.ySpeakerPosM), Math.abs(x - FieldConstants.xSpeakerPosM)));
    }
  }
}
