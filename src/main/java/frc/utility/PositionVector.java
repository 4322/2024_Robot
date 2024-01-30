package frc.utility;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PositionVector {
  private double distance;
  private double angleRadians;
  private Rotation2d angle;
  private Translation2d vector;
  private double xDistToTarget;
  private double yDistToTarget;

  private double xSpeakerPosM;
  private double ySpeakerPosM;

  // 218.353069 in
  public Translation2d getVectorToSpeaker(double x, double y) { // Position of robot relative to origin ()
    if (DriverStation.getAlliance().get().equals(Alliance.Blue)) { // Account for origin remaining same between blue and red
      xSpeakerPosM = 0;
      ySpeakerPosM = 5.546;
    } else {
      xSpeakerPosM = 16.591;
      ySpeakerPosM = 5.546;
    }

    distance = Math.sqrt(((x - xSpeakerPosM) * (x - xSpeakerPosM)) + 
                         ((y - ySpeakerPosM) * (y - ySpeakerPosM)));

    xDistToTarget = Math.abs(x - xSpeakerPosM);
    yDistToTarget = Math.abs(y - ySpeakerPosM);

    angleRadians = Math.atan2(yDistToTarget, xDistToTarget);

    

    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      angleRadians = -(angleRadians - Math.PI) + Math.PI; // Flip by 1 radian if red
    }

    angle = new Rotation2d(angleRadians); // angle in radians 
    vector = new Translation2d(distance, angle); 
    return vector;
  }
}
