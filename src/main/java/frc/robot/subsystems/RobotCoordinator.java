package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.intakeDeployer.IntakeDeployer;
import frc.robot.subsystems.intakeDeployer.IntakeDeployer.IntakeDeployerStates;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtakePivot.OuttakePivot;

public class RobotCoordinator extends SubsystemBase {

  public enum RobotStates {
    defaultDrive, 
    deployRequested,
    retractRequested,
    noteObtained,
    noteSecured,
    huntingNote;
  }

  public enum HuntingStates {
    manual,
    auto;
  }

  private RobotStates robotState = RobotStates.defaultDrive;
  private HuntingStates huntingState = HuntingStates.manual;

  private Intake intake;
  private Outtake outtake;
  private IntakeDeployer intakeDeployer;
  private OuttakePivot outtakePivot;
  private Drive drive;

  private static BeamBreakSensorIO noteTrackerSensorsIO;
  private static BeamBreakSensorIOInputsAutoLogged inputs = new BeamBreakSensorIOInputsAutoLogged();

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
        if (Constants.sensorsEnabled) {
          noteTrackerSensorsIO = new BeamBreakSensorIOReal();
        }
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }

    if (noteTrackerSensorsIO == null) {
      noteTrackerSensorsIO = new BeamBreakSensorIO() {};
    }
  }

  public RobotStates getRobotState() {
    return robotState;
  }

  public void setRobotState(RobotStates newState) {
    robotState = newState;
  }

  @Override
  public void periodic() {
    noteTrackerSensorsIO.updateInputs(inputs);

    switch (robotState) {
      case defaultDrive:
        break;
      case deployRequested:
        if (canDeploy()) {
          intakeDeployer.setState(IntakeDeployerStates.deploying);
        }
        if (!noteInRobot() && intakeDeployer.getState() == IntakeDeployerStates.deployed) {
          robotState = RobotStates.huntingNote;
        }
        break;
      case huntingNote:
        if (canIntake()) {
          intake.setState(IntakeStates.feeding);
        }
        switch (huntingState) {
          case manual:
            break;
          case auto:
            break;
        }
        if (noteAtIntakeSensor()) {
          robotState = RobotStates.noteObtained;
        }
        break;
      case noteObtained:
        if (noteInRobot()) {
          robotState = RobotStates.noteSecured;
        }
        break;
      case noteSecured:
        intake.setState(IntakeStates.stopped);
        if (!isAcrossCenterLine()) {
          robotState = RobotStates.retractRequested;
        }
        break;
      case retractRequested:
        intake.setState(IntakeStates.stopped);
        if (canRetract()) {
          intakeDeployer.setState(IntakeDeployerStates.retracting);
        }
        if (isIntakeRetracted() && intakeDeployer.getState() == IntakeDeployerStates.retracted) {
          robotState = RobotStates.defaultDrive;
        }
        break;
    }
  }

  // below are all boolean checks polled from subsystems
  public boolean canIntake() {
    return intakeDeployer.isAtPosition() && intakeDeployer.isDeployed();
  }

  public boolean isIntakeDeployed() {
    return intakeDeployer.isDeployed();
  }

  public boolean isIntakeRetracted() {
    return intakeDeployer.isRetracted();
  }

  public boolean canDeploy() {
    return intakeDeployer.isInitialized() && !intakeDeployer.isDeployed();
  }

  public boolean canRetract() {
    return intake.getState() == IntakeStates.stopped && inputs.intakeBeamBreak;
  }

  public boolean canShoot() {
    return outtake.isFlyWheelUpToSpeed()
        && outtakePivot.isAtPosition()
        && noteInRobot();
  }

  public boolean canPivot() {
    return outtakePivot.isInitialized();
  }

  public boolean intakingNote() {
    return !inputs.intakeBeamBreak && intake.getState() == IntakeStates.feeding;
  }

  public boolean noteInRobot() {
    return inputs.intakeBeamBreak && !inputs.tunnelBeamBreak;
  }

  public boolean noteAtIntakeSensor() {
    return !inputs.intakeBeamBreak;
  }

  public boolean noteAtTunnelSensor() {
    return !inputs.tunnelBeamBreak;
  }

  public boolean isAcrossCenterLine() {
    if (Robot.getAllianceColor().equals(Alliance.Red)) {
      return (drive.getPose2d().getX() > Constants.FieldConstants.xCenterLineM);
    } else if (Robot.getAllianceColor().equals(Alliance.Blue)) {
      return (drive.getPose2d().getX() < Constants.FieldConstants.xCenterLineM);
    } else {
      return false;
    }
  }

  public double getRobotXPos() {
    return drive.getPose2d().getX();
  }

  public double getRobotYPos() {
    return drive.getPose2d().getY();
  }
}
