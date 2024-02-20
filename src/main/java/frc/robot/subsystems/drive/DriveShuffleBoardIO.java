package frc.robot.subsystems.drive;

import frc.robot.RobotChooser.RobotChooser;
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
    public double fastMovingMetersPerSec;
    public double accelerationRampRate;
    public double stoppedRampRate;
    public double[] voltsOverMetersPerSecAtSpeedThresholds =
        new double
            [RobotChooser.getInstance().getConstants().getDriveffVoltsOverMetersPerSec().length];
    public double[] feedForwardMetersPerSecThresholds =
        new double
            [RobotChooser.getInstance()
                .getConstants()
                .getDriveffSpeedMetersPerSecThresholds()
                .length];
    public double voltsToOvercomeFriction;
  }

  public default void updateInputs(DriveShuffleBoardIOInputs inputs) {}
}
