package frc.robot.subsystems.outtake;

public interface OuttakeInterface {
  public void outtake(double targetRPM);

  public void stopOuttake();

  public void setCoastMode();

  public void setBrakeMode();

  public boolean isFlyWheelUpToSpeed();

  public double getTargetRPM();
}
