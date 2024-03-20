package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double rightIntakeRotations = 0.0;
    public double rightIntakeRotationsPerSec = 0.0;
    public double rightIntakeAppliedVolts = 0.0;
    public double rightIntakeSupplyCurrentAmps = 0.0;
    public double rightIntakeStatorCurrentAmps = 0.0;
    public double rightIntakeTempC = 0.0;
    public boolean rightIntakeIsAlive = false;
    public double rightIntakeSpeedPct = 0.0;

    public double leftIntakeRotations = 0.0;
    public double leftIntakeRotationsPerSec = 0.0;
    public double leftIntakeAppliedVolts = 0.0;
    public double leftIntakeSupplyCurrentAmps = 0.0;
    public double leftIntakeStatorCurrentAmps = 0.0;
    public double leftIntakeTempC = 0.0;
    public boolean leftIntakeIsAlive = false;
    public double leftIntakeSpeedPct = 0.0;

    public double intakeFeederVoltage = 0.0;
    public double intakeEjectVoltage = 0.0;

    public double deployRotations = 0.0;
    public double deployRotationsPerSec = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deploySupplyCurrentAmps = 0.0;
    public double deployStatorCurrentAmps = 0.0;
    public double deployTempC = 0.0;
    public boolean deployIsAlive = false;

    public double heliumAbsRotations = 0.0;
    public double heliumRPS = 0.0;

    public String deployAppliedControl = "";
    public double deployMaxRotationsPerSec;
    public double deployKp = 0.0;
    public double slowPos = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setFeedingVoltage(double voltage) {}

  public default boolean initMotorPos() {
    return false;
  }

  public default void setDeployVoltage(double voltage) {}

  public default void setDeployKp(double kP) {}

  public default void setIntakeBrakeMode() {}

  public default void setIntakeCoastMode() {}

  public default void setDeployerBrakeMode() {}

  public default void setDeployerCoastMode() {}

  public default void stopFeeder() {}

  public default void stopDeployer() {}
}
