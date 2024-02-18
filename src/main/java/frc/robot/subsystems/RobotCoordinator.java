package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtakePivot.OuttakePivot;

public class RobotCoordinator extends SubsystemBase {
  private Intake intake;
  private Outtake outtake;
  private OuttakePivot outtakePivot;
  private Drive drive;

  private static BeamBreakSensorIO noteTrackerSensorsIO;
  private static BeamBreakSensorIOInputsAutoLogged inputs = new BeamBreakSensorIOInputsAutoLogged();

  private static RobotCoordinator robotCoordinator;

  private boolean intakeButtonPressed;

  public static RobotCoordinator getInstance() {
    if (robotCoordinator == null) {
      robotCoordinator = new RobotCoordinator();
    }
    return robotCoordinator;
  }

  private RobotCoordinator() {
    intake = Intake.getInstance();
    outtake = Outtake.getInstance();
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

  @Override
  public void periodic() {
    noteTrackerSensorsIO.updateInputs(inputs);
  }

  public void setIntakeButtonState(boolean isPressed) {
    intakeButtonPressed = isPressed;
  }

  public boolean getIntakeButtonPressed() {
    return intakeButtonPressed;
  }

  // below are all boolean checks polled from subsystems
  public boolean canIntake() {
    return intake.isAtPosition() && intake.isDeployed();
  }

  public boolean isIntakeDeployed() {
    return intake.isDeployed();
  }

  public boolean isIntakeRetracted() {
    return intake.isRetracted();
  }

  public boolean canDeploy() {
    return intake.isInitialized() && !intake.isDeployed();
  }

  public boolean canShoot() {
    return outtake.isFlyWheelUpToSpeed()
        && outtakePivot.isAtPosition()
        && noteInTunnel();
  }

  public boolean canPivot() {
    return outtakePivot.isInitialized();
  }

  public boolean intakingNote() {
    return !inputs.intakeBeamBreak && intake.getState() == IntakeStates.feeding;
  }

  public boolean noteInTunnel() {
    return inputs.intakeBeamBreak && !inputs.tunnelBeamBreak;
  }

  public boolean noteInIntake() {
    return !inputs.intakeBeamBreak;
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
