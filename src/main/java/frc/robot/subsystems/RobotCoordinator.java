package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BeamBreakConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtakePivot.OuttakePivot;
import org.littletonrobotics.junction.Logger;

public class RobotCoordinator extends SubsystemBase {
  private Intake intake;
  private Outtake outtake;
  private OuttakePivot outtakePivot;
  private Drive drive;

  private static BeamBreakSensorIO noteTrackerSensorsIO;
  private static BeamBreakSensorIOInputsAutoLogged inputs = new BeamBreakSensorIOInputsAutoLogged();

  private static RobotCoordinator robotCoordinator;
  private Timer shootTimer = new Timer();

  private boolean intakeButtonPressed;
  private boolean notePassingIntake;
  private boolean notePassingTunnel;
  private boolean autoIntakeButtonPressed;

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
    if (Constants.sensorsEnabled) {
      noteTrackerSensorsIO.updateInputs(inputs);
      Logger.processInputs(BeamBreakConstants.Logging.key, inputs);
    }

    // update note tracking logic in robot
    if (!inputs.intakeBeamBreak) {
      notePassingIntake = true;
    } else if (!inputs.tunnelBeamBreak) {
      notePassingIntake = false;
      notePassingTunnel = true;
    } else if (inputs.tunnelBeamBreak && notePassingTunnel) {
      shootTimer.start();
      if (shootTimer.hasElapsed(0.5)) {
        notePassingTunnel = false;
        shootTimer.stop();
        shootTimer.reset();
      }
    }
  }

  public void setIntakeButtonState(boolean isPressed) {
    intakeButtonPressed = isPressed;
  }

  public boolean getIntakeButtonPressed() {
    return intakeButtonPressed
        || getAutoIntakeButtonPressed(); // auto intake button is an identical bind so it also
    // counts as an intake button
  }

  public void setAutoIntakeButtonPressed(boolean isPressed) {
    autoIntakeButtonPressed = isPressed;
  }

  public boolean getAutoIntakeButtonPressed() {
    return autoIntakeButtonPressed;
  }

  // below are all boolean checks polled from subsystems
  public boolean isIntakeDeployed() {
    return intake.isDeployed();
  }

  public boolean isIntakeRetracted() {
    return intake.isRetracted();
  }

  public boolean canDeploy() {
    return intake.isInitialized() && !intake.isDeployed();
  }

  public boolean canRetract() {
    return !intake.isFeeding() && intake.isInitialized();
  }

  public boolean canShoot() {
    return outtake.isFlyWheelUpToSpeed() && outtakePivot.isAtPosition() && noteInFiringPosition();
  }

  public boolean canPivot() {
    return outtakePivot.isInitialized();
  }

  public boolean noteInFiringPosition() {
    return !inputs.tunnelBeamBreak;
  }

  public boolean noteInIntake() {
    return !inputs.intakeBeamBreak;
  }

  public boolean noteInRobot() {
    return !inputs.intakeBeamBreak
        || !inputs.tunnelBeamBreak
        || notePassingIntake
        || notePassingTunnel;
  }

  public boolean noteIsShot() {
    return !notePassingTunnel && inputs.tunnelBeamBreak;
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

  public Double getNearestNoteTX() {
    throw new UnsupportedOperationException("Unimplemented method 'getNearestNoteTX'");
  }

  public Double getNearestNoteTY() {
    throw new UnsupportedOperationException("Unimplemented method 'getNearestNoteTY'");
  }
}
