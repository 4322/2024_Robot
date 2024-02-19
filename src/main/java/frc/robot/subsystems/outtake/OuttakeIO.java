package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public class OuttakeIOInputs {
    public double topCurrentAmps = 0.0;
    public double topTempC = 0.0;
    public double topRotationsPerSec = 0.0;

    public double bottomCurrentAmps = 0.0;
    public double bottomTempC = 0.0;
    public double bottomRotationsPerSec = 0.0;

    public boolean topOuttakeIsAlive = false;
    public boolean bottomOuttakeIsAlive = false;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}
  ;

  public default void setOuttakeRPS(
      double desiredTopVelocityRPS, double desiredBottomVelocityRPS) {}
  ;

  public default void setBrakeMode() {}
  ;

  public default void setCoastMode() {}
  ;

  public default void stopOuttake() {}
  ;
}
