package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NoteTrackerInterface;
import frc.robot.subsystems.intake.IntakeInterface;
import frc.robot.subsystems.intakeDeployer.IntakeDeployerInterface;
import frc.robot.subsystems.outtake.OuttakeInterface;
import frc.robot.subsystems.outtakePivot.OuttakePivotInterface;

public class RobotCoordinator extends SubsystemBase implements RobotCoordinatorInterface {
  private IntakeInterface intake;
  private OuttakeInterface outtake;
  private IntakeDeployerInterface intakeDeployer;
  private OuttakePivotInterface outtakePivot;
  private NoteTrackerInterface noteTracker;

  public RobotCoordinator(
      IntakeInterface intake,
      OuttakeInterface outtake,
      IntakeDeployerInterface deployer,
      OuttakePivotInterface pivot,
      NoteTrackerInterface noteTracker) {
    this.intake = intake;
    this.outtake = outtake;
    intakeDeployer = deployer;
    outtakePivot = pivot;
    this.noteTracker = noteTracker;
  }

  @Override
  public void periodic() {}

  public boolean canIntake() {
    return intakeDeployer.isAtPosition() && intakeDeployer.isDeployed();
  }

  public boolean canDeploy() {
    return intakeDeployer.isInitialized() && !intakeDeployer.isDeployed();
  }

  public boolean hasNote() {
    return noteTracker.hasNote();
  }

  public boolean noteInFiringPosition() {
    return noteTracker.isInPosition();
  }

  public boolean canShoot() {
    return outtake.isFlyWheelUpToSpeed() && outtakePivot.isAtPosition() && hasNote();
  }

  public boolean canPivot() {
    return outtakePivot.isInitialized();
  }
}
