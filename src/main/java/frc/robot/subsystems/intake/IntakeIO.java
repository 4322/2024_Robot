package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double intakeRotations = 0.0;
    public double intakeRotationsPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;

    public double turnVelocityDegPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnDegrees = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setBrakeMode() {}

  public default void setCoastMode() {}
  
}
