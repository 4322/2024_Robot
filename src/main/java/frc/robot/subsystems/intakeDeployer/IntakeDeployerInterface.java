package frc.robot.subsystems.intakeDeployer;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeDeployerInterface extends Subsystem {
  public void deploy();

  public void retract();

  public boolean isAtPosition();

  public void setBrakeMode();

  public void setCoastMode();

  public void stopDeploy();

  public boolean isDeployed();

  public boolean isInitialized();
}
