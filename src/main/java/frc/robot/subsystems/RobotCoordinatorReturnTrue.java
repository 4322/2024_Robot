package frc.robot.subsystems;

public class RobotCoordinatorReturnTrue implements RobotCoordinatorInterface {
  // FOR TESTING ONLY
  public boolean canDeploy() {
    return true;
  }

  public boolean canIntake() {
    return true;
  }

  public boolean canPivot() {
    return true;
  }

  public boolean canShoot() {
    return true;
  }

  public boolean hasNote() {
    return true;
  }

  public boolean noteInFiringPosition() {
    return true;
  }
}
