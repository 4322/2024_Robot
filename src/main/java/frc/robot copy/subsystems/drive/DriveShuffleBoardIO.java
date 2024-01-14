package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveShuffleBoardIO {
  @AutoLog
  public static class DriveShuffleBoardIOInputs {
    public boolean pseudoAutoRotateEnabled;
    public String inputScaling;
    public String driveControllerType; 
    public double maxManualRotatePower;
    public double slowMovingAutoRotatePower;
    public double fastMovingAutoRotatePower;
    public double fastMovingFtPerSec;
    public double accelerationRampRate;
    public double stoppedRampRate;
    public double[] voltsAtSpeedThresholds;
    public double[] feedForwardRPSThresholds;
    public double voltsToOvercomeFriction;
    
  }

  public default void updateInputs(DriveShuffleBoardIOInputs inputs) {}
}
