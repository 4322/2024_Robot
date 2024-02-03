package frc.robot.subsystems.intakeDeployer;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeDeployerIO {
  @AutoLog
  public class IntakeDeployerIOInputs {
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

  public default void updateInputs(IntakeDeployerIOInputs inputs) {}

  public default boolean initMotorPos() {
    return false;
  }

  public default void setDeployTarget(double rotations) {}

  public default void setBrakeMode() {}

  public default void setCoastMode() {}

  public default void stopDeploy() {}
}
