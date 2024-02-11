package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NoteTracker.NoteTracker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeDeployer.IntakeDeployer;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtakePivot.OuttakePivot;

public class RobotCoordinator extends SubsystemBase {
  private Intake intake;
  private Outtake outtake;
  private IntakeDeployer intakeDeployer;
  private OuttakePivot outtakePivot;
  private NoteTracker noteTracker;

  private static RobotCoordinator robotCoordinator;

  public static RobotCoordinator getInstance() {
    if (robotCoordinator == null) {
      robotCoordinator = new RobotCoordinator();
    }
    return robotCoordinator;
  }

  private RobotCoordinator() {
    intake = Intake.getInstance();
    outtake = Outtake.getInstance();
    intakeDeployer = IntakeDeployer.getInstance();
    outtakePivot = OuttakePivot.getInstance();
    noteTracker = NoteTracker.getInstance();
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
