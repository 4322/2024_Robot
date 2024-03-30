package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public class OuttakeIOInputs {
    public double topCurrentAmps = 0.0;
    public double topTempC = 0.0;
    public double topRotationsPerSec = 0.0;
    public double topDebugTargetRPS = 0.0;

    public double bottomSupplyCurrentAmps = 0.0;
    public double bottomStatorCurrentAmps = 0.0;
    public double bottomTempC = 0.0;
    public double bottomRotationsPerSec = 0.0;
    public double bottomDebugTargetRPS = 0.0;

    public double pivotRotations = 0.0;
    public double pivotRotationsPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotTempC = 0.0;

    public double pivotEncoderRotations = 0.0;
    public double pivotEncoderRotationsPerSec = 0.0;
    public double targetPivotPosition;

    public boolean topOuttakeIsAlive = false;
    public boolean bottomOuttakeIsAlive = false;
    public boolean pivotIsAlive = false;

    public double heliumAbsRotations = 0.0;
    public double heliumRelativeRotations = 0.0;

    public boolean tuneOuttakeOverrideEnable = false;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}
  ;

  public default void setOuttakeRPS(
      double desiredTopVelocityRPS, double desiredBottomVelocityRPS) {}
  ;

  public default void setPivotTarget(double rotations) {}

  public default void setPivotBrakeMode() {}
  ;

  public default void setPivotCoastMode() {}
  ;

  public default void setFlywheelCoastMode() {}
  ;

  public default void stopOuttake() {}
  ;

  public default void stopPivot() {}
  ;

  public default boolean pivotIsInitialized() {return false;}
}
