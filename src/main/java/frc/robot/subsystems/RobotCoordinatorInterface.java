package frc.robot.subsystems;

public interface RobotCoordinatorInterface {
  public boolean canIntake();

  public boolean canDeploy();

  public boolean canShoot();

  public boolean canPivot();

  public boolean hasNote();

  public boolean noteInFiringPosition();
}
