package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.Constants.DriveConstants;

public interface DriveShuffleBoardIO {
  @AutoLog
  public static class DriveShuffleBoardIOInputs {
    public boolean psuedoAutoRotateEnabled;
    public String inputScaling;
    public String driveControllerType; 
    public double maxManualRotatePower;
    public double slowMovingAutoRotatePower;
    public double fastMovingAutoRotatePower;
    public double fastMovingFtPerSec;
    public double accelerationRampRate;
    public double stoppedRampRate;
    public double[] voltsOverMetersPerSecAtSpeedThresholds = new double[DriveConstants.Drive.FeedForward.voltsOverMetersPerSecAtSpeedThresholds.length];
    public double[] feedForwardMetersPerSecThresholds = new double[DriveConstants.Drive.FeedForward.feedForwardMetersPerSecThreshold.length];
    public double voltsToOvercomeFriction;
    
  }

  public default void updateInputs(DriveShuffleBoardIOInputs inputs) {}
}
