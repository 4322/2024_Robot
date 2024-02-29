package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public class OuttakeIOInputs {
    public double leftCurrentAmps = 0.0;
    public double leftTempC = 0.0;
    public double leftRotationsPerSec = 0.0;
    public double debugTargetRPS = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTempC = 0.0;
    public double rightRotationsPerSec = 0.0;

    public double pivotRotations = 0.0;
    public double pivotRotationsPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
    public double pivotTempC = 0.0;

    public double pivotEncoderRotations = 0.0;
    public double pivotEncoderRotationsPerSec = 0.0;
    public double targetPivotPosition;

    public boolean leftOuttakeIsAlive = false;
    public boolean rightOuttakeIsAlive = false;
    public boolean pivotIsAlive = false;

    public double heliumAbsRotations = 0.0;
    public double heliumRelativeRotations = 0.0;

    public boolean debugOuttakeEnabled = false;
    public boolean debugPivotEnabled = false;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}
  ;

  public default void setOuttakeRPS(
      double desiredTopVelocityRPS, double desiredBottomVelocityRPS) {}
  ;

  public default boolean initPivot() {
    return false;
  }

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
}
