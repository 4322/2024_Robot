package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public boolean calibrating = false; // for NavX only
    public double rollPositionDeg = 0.0;
    public double pitchPositionDeg = 0.0;
    public double yawPositionDeg = 0.0;
    public double accelX = 0.0; // Pigeon only
    public double accelY = 0.0; // Pigeon only
    public double accelZ = 0.0; // Pigeon only
    public double yawVelocityDegPerSec = 0.0;
    public double yawAngleDeg = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void reset() {}
}
