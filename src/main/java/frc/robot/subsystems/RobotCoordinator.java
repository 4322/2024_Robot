package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BeamBreakConstants;
import frc.robot.Robot;
import frc.robot.commands.IntakeManual;
import frc.robot.commands.IntakeManual.IntakeStates;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.outtake.Outtake;
import org.littletonrobotics.junction.Logger;

public class RobotCoordinator extends SubsystemBase {
  private Intake intake = Intake.getInstance();
  private Outtake outtake = Outtake.getInstance();
  private Drive drive = Drive.getInstance();
  private Limelight outtakeLimelight = Limelight.getOuttakeInstance();
  private Limelight intakeLimelight = Limelight.getIntakeInstance();
  private Climber climber = Climber.getInstance();
  private static BeamBreakSensorIO noteTrackerSensorsIO;
  private static BeamBreakSensorIOInputsAutoLogged inputs = new BeamBreakSensorIOInputsAutoLogged();

  private static RobotCoordinator robotCoordinator;
  private Timer shootTimer = new Timer();

  private boolean intakeButtonPressed;
  private boolean slowClimbButtonHeld;
  private boolean notePassingIntake;
  private boolean notePassingTunnel;
  private boolean autoIntakeButtonPressed;
  private boolean initAbsEncoderPressed;
  private boolean outtakeInClimbState = false;

  public static RobotCoordinator getInstance() {
    if (robotCoordinator == null) {
      robotCoordinator = new RobotCoordinator();
    }
    return robotCoordinator;
  }

  private RobotCoordinator() {
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
    if (!inputs.intakeBeamBreak && intakeIsFeeding()) {
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
    } else if (intakeIsEjecting()) {
      notePassingIntake = false;
    }
  }

  public void setIntakeButtonState(boolean isPressed) {
    intakeButtonPressed = isPressed;
  }

  public boolean isClimbing() {
    return outtakeInClimbState;
  }

  public void setInClimbingMode(boolean inClimbState) {
    outtakeInClimbState = inClimbState;
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

  public void setInitAbsEncoderPressed(boolean isPressed) {
    initAbsEncoderPressed = isPressed;
  }

  public boolean getInitAbsEncoderPressed() {
    return initAbsEncoderPressed;
  }

  // below are all boolean checks polled from subsystems
  public boolean isIntakeDeployed() {
    return intake.isDeployed();
  }

  public boolean isIntakeDeploying() {
    return intake.isDeploying();
  }

  public double getDeployRotations() {
    return intake.getDeployRotations();
  }

  public boolean isIntakeRetracted() {
    return intake.isRetracted();
  }

  public boolean canDeploy() {
    return !intake.isDeployed();
  }

  public boolean intakeIsDeployed() {
    return intake.isDeployed();
  }

  public IntakeStates getIntakeState() {
    return IntakeManual.getIntakeState();
  }

  public boolean isInitialized() {
    return outtake.pivotIsInitialized();
  }

  public boolean canRetract() {
    return !intake.isFeeding() && !intake.isEjecting();
  }

  public boolean intakeIsFeeding() {
    return intake.isFeeding();
  }

  public boolean intakeIsEjecting() {
    return intake.isEjecting();
  }

  public boolean canShoot() {
    return outtake.isFlyWheelUpToSpeed() && outtake.pivotIsAtPosition();
  }

  // TODO: add checks to this
  public boolean canSpinFlywheel() {
    return true;
  }

  public boolean canPivot() {
    return outtake.pivotIsInitialized();
  }

  public boolean noteInFiringPosition() {
    return !inputs.tunnelBeamBreak;
  }

  public boolean noteEnteringIntake() {
    return !inputs.intakeBeamBreak && intake.isFeeding();
  }

  public boolean noteEjectingThroughIntake() {
    return !inputs.intakeBeamBreak && intake.isEjecting();
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

  public boolean noteInIntake() {
    return !inputs.intakeBeamBreak;
  }

  public boolean onOurSideOfField() {
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
    return intakeLimelight.getHorizontalDegToTarget();
  }

  public Double getNearestNoteTY() {
    return intakeLimelight.getVerticalDegToTarget();
  }

  public boolean noteInVision() {
    return intakeLimelight.getTargetVisible();
  }

  public boolean pivotAtPosition() {
    return outtake.pivotIsAtPosition();
  }

  public boolean climberIsFullyRetracted() {
    return (climber.isFullyRetracted());
  }

  public boolean climberIsFullyExtended() {
    return (climber.isFullyExtended());
  }

  public boolean debugOuttakeOverride() {
    return outtake.getDebugOverrideEnabled();
  }
}
