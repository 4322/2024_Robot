package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double intakeRotations = 0.0;
    public double intakeRotationsPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double intakeTempC = 0.0;
    public boolean intakeIsAlive = false;

    public double intakeSpeedPct = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntake(double pct) {}

  public default void setBrakeMode() {}

  public default void setCoastMode() {}

  public default void stopIntake() {}
}
