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
    public double intakeFeederVoltage = 0.0;
    public double intakeEjectVoltage = 0.0;
    public double intakeSpeedPct = 0.0;

    public double deployRotations = 0.0;
    public double deployRotationsPerSec = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;
    public double deployTempC = 0.0;
    public boolean deployIsAlive = false;

    public double heliumRotations = 0.0;
    public double heliumRPS = 0.0;

    public String deployAppliedControl = "";
    public double deployKp;
    public double slowPos;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setFeedingVoltage(double voltage) {}

  public default boolean initMotorPos() {
    return false;
  }

  public default void setDeployVoltage(double voltage) {}
  public default void setDeployKp(double kP)
  {}
  public default void setIntakeBrakeMode() {}

  public default void setIntakeCoastMode() {}

  public default void setDeployerBrakeMode() {}

  public default void setDeployerCoastMode() {}

  public default void stopFeeder() {}

  public default void stopDeployer() {}
}
