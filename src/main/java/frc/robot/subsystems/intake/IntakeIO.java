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

    public double deployRotations = 0.0;
    public double deployRotationsPerSec = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;
    public double deployTempC = 0.0;
    public boolean deployIsAlive = false;

    public double deployEncoderRotations = 0.0;
    public double deployEncoderRotationsPerSec = 0.0;

    public String deployAppliedControl = "";
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeRPM(double rpm) {}

  public default boolean initMotorPos() {
    return false;
  }

  public default void setDeployTarget(double rotations) {}
  
  public default void setBrakeMode() {}

  public default void setCoastMode() {}

  public default void stopIntake() {}
}
