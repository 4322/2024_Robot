package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.utility.OrangeMath;

public class DriveManualLogic {
  
  public static Translation2d doDriveNormalizationScaling(double driveRawX, double driveRawY, double driveDeadband, int power) {

    final double driveRawMag = OrangeMath.pythag(driveRawX, driveRawY);
    final double driveRawTheta = Math.atan2(driveRawY, driveRawX);

    // Normalize the drive input over deadband in polar coordinates.
    double driveMag = 0;
    if (driveRawMag > driveDeadband) {
      driveMag = (driveRawMag - driveDeadband) / (1 - driveDeadband);

      if (Constants.driveTuningMode) {
        // quantize input drive magnitude to 0, 0.25, 0.5, 0.75, 1.0 for PID tuning
        driveMag = Math.round(driveMag * 4.0) / 4.0;
      }

      switch (power) {
        case 1:
          break;
        case 2:
          driveMag = driveMag * driveMag;
          break;
        case 3:
          driveMag = driveMag * driveMag * driveMag;
          break;
        default: // not 1-3
          break;
      }
    }
    // Convert back to cartesian coordinates
    double driveX = Math.cos(driveRawTheta) * driveMag;
    double driveY = Math.sin(driveRawTheta) * driveMag;
    
    // Normalize the combined drive vector
    if (driveX > 1) {
      driveY /= driveX;
      driveX = 1;
    } else if (driveX < -1) {
      driveY /= -driveX;
      driveX = -1;
    }
    if (driveY > 1) {
      driveX /= driveY;
      driveY = 1; 
    } else if (driveY < -1) {
      driveX /= -driveY;
      driveY = -1;
    }

    return new Translation2d(driveX, driveY);
  }
}
