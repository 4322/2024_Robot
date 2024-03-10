package frc.utility;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;

public class FiringSolutionHelper {
  // 218.353069 inches
  private static double distance;
  private static double angleRadians;
  private static Rotation2d angle;
  private static Translation2d vector;
  private static double xComponentToTarget;
  private static double yComponentToTarget;
  private static Translation2d botTranslation2d;
  private static Translation2d speakerTranslation2d;

  public static Translation2d getVectorToSpeaker(
      double x, double y) { // Position of robot relative to origin ()
    botTranslation2d = new Translation2d(x, y);
    if (Robot.isRed()) {
      speakerTranslation2d = FieldConstants.redSpeakerTranslation2d;
    } else {
      speakerTranslation2d = FieldConstants.blueSpeakerTranslation2d;
    }

    distance = botTranslation2d.getDistance(speakerTranslation2d);

    xComponentToTarget = Math.abs(x - speakerTranslation2d.getX());
    yComponentToTarget = y - speakerTranslation2d.getY();

    angleRadians = Math.atan2(yComponentToTarget, xComponentToTarget);

    if (Robot.isRed()) {
      angleRadians = -angleRadians; // Flip by pi radian if red
      // Assumes that 0 is alwyas facing away from speaker
    }

    angle = new Rotation2d(angleRadians); // angle in radians
    vector = new Translation2d(distance, angle);
    return vector;
  }

  public static Translation2d getSpeakerTranslation2d() {
    if (Robot.isRed()) {
      return FieldConstants.redSpeakerTranslation2d;
    } else {
      return FieldConstants.blueSpeakerTranslation2d;
    }
  }
}
