package frc.robot.subsystems.outtakePivot;

public interface OuttakePivotInterface {
  public void pivot(double rotations);

  public void resetPivot();

  public void stopPivot();

  public void setCoastMode();

  public void setBrakeMode();

  public boolean isAtPosition();

  public boolean isInitialized();
}
