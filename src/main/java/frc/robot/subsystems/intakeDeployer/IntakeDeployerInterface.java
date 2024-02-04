package frc.robot.subsystems.intakeDeployer;

public interface IntakeDeployerInterface {
  public void deploy();

  public void retract();

  public boolean isAtPosition();

  public void setBrakeMode();

  public void setCoastMode();

  public void stopDeploy();

  public boolean isDeployed();

  public boolean isInitialized();
}
