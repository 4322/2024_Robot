package frc.robot.subsystems.intake;

public interface IntakeInterface {
  // used primarily for reference when writing the intake
  public void intake();
  public void outtake();
  public void deploy();
  public void undeploy();
  public boolean isAtPosition();
  public void setBrakeMode();
  public void setCoastMode();
  public void stopIntake();
  public void stopDeploy();
}
