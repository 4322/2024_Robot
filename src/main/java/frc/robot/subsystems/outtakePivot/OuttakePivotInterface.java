package frc.robot.subsystems.outtakePivot;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface OuttakePivotInterface extends Subsystem {
  public void pivot(double rotations);

  public void resetPivot();

  public void stopPivot();

  public void setCoastMode();

  public void setBrakeMode();

  public boolean isAtPosition();

  public boolean isInitialized();

  public double getTarget();
}
