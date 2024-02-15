package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.intakeDeployer.IntakeDeployer;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtakePivot.OuttakePivot;
import frc.utility.OrangeMath;

public class RobotCoordinator extends SubsystemBase {

  public enum RobotStates {
    defaultDrive, 
    deploying,
    feeding,
    stowing,
    noteObtained,
    noteSecured,
    readyToShoot,
    shoot;
  }

  public enum FeedingStates {
    manual,
    auto;
  }

  private RobotStates robotState = RobotStates.defaultDrive;

  private Intake intake;
  private Outtake outtake;
  private IntakeDeployer intakeDeployer;
  private OuttakePivot outtakePivot;
  private Drive drive;

  private static DistanceSensorIO noteTrackerSensorsIO;
  private static DistanceSensorIOInputsAutoLogged inputs = new DistanceSensorIOInputsAutoLogged();

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
    drive = Drive.getInstance();
    switch (Constants.currentMode) {
      case REAL:
        noteTrackerSensorsIO = new DistanceSensorIOReal();
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }

    if (noteTrackerSensorsIO == null) {
      noteTrackerSensorsIO = new DistanceSensorIO() {};
    }
  }

  @Override
  public void periodic() {
    noteTrackerSensorsIO.updateInputs(inputs);
  }

  public RobotStates getRobotState() {
    return robotState;
  }

  public boolean canIntake() {
    return intakeDeployer.isAtPosition() && intakeDeployer.isDeployed();
  }

  public boolean isIntakeDeployed() {
    return intakeDeployer.isDeployed();
  }

  public boolean canDeploy() {
    return intakeDeployer.isInitialized() && !intakeDeployer.isDeployed();
  }

  public boolean canShoot() {
    return outtake.isFlyWheelUpToSpeed()
        && outtakePivot.isAtPosition()
        && noteInFiringPos();
  }

  public boolean canPivot() {
    return outtakePivot.isInitialized();
  }

  public boolean intakingNote() {
    return inputs.intakeBeamBreak && intake.getState() == IntakeStates.INTAKING;
  }

  public boolean noteInRobot() {
    return !inputs.intakeBeamBreak && inputs.tunnelBeamBreak;
  }

  public boolean noteAtFirstSensor() {
    return inputs.intakeBeamBreak;
  }

  public boolean noteInFiringPos() {
    return OrangeMath.equalToTwoDecimal(
      inputs.tunnelDistance, Constants.TunnelConstants.noteToSensorDistMeters);
  }

  public boolean isAcrossCenterLine() {
    if (Robot.getAllianceColor().equals(Alliance.Red)) {
      return (drive.getPose2d().getX() > Constants.FieldConstants.xCenterLineM);
    } else {
      return (drive.getPose2d().getX() < Constants.FieldConstants.xCenterLineM);
    }
  }

  public double getRobotXPos() {
    return drive.getPose2d().getX();
  }

  public double getRobotYPos() {
    return drive.getPose2d().getY();
  }
}
