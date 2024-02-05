package frc.robot.subsystems;

public class TestRobotCoordinator implements RobotCoordinatorInterface {
  // FOR TESTING ONLY
  private boolean canDeploy;
  private boolean canIntake;
  private boolean canPivot;
  private boolean canShoot;
  private boolean hasNote;
  private boolean noteInFiringPosition;

  public TestRobotCoordinator(
      boolean canDeploy,
      boolean canIntake,
      boolean canPivot,
      boolean canShoot,
      boolean hasNote,
      boolean noteInFiringPosition) {
    this.canDeploy = canDeploy;
    this.canIntake = canIntake;
    this.canPivot = canPivot;
    this.canShoot = canShoot;
    this.hasNote = hasNote;
    this.noteInFiringPosition = noteInFiringPosition;
  }

  public boolean canDeploy() {
    return canDeploy;
  }

  public void setCanDeploy(boolean canDeploy) {
    this.canDeploy = canDeploy;
  }

  public boolean canIntake() {
    return canIntake;
  }

  public void setCanIntake(boolean canIntake) {
    this.canIntake = canIntake;
  }

  public boolean canPivot() {
    return canPivot;
  }

  public void setCanPivot(boolean canPivot) {
    this.canPivot = canPivot;
  }

  public boolean canShoot() {
    return canShoot;
  }

  public void setCanShoot(boolean canShoot) {
    this.canShoot = canShoot;
  }

  public boolean hasNote() {
    return hasNote;
  }

  public void setHasNote(boolean hasNote) {
    this.hasNote = hasNote;
  }

  public boolean noteInFiringPosition() {
    return noteInFiringPosition;
  }

  public void setNoteInFiringPosition(boolean noteInFiringPosition) {
    this.noteInFiringPosition = noteInFiringPosition;
  }
}
