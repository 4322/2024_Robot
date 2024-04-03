package frc.robot.commands.DriveManual;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.Manual;
import frc.robot.Constants.DriveInputScalingStrings;
import frc.robot.Constants.RotateInputScalingStrings;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualState;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualTrigger;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.Limelight;
import frc.utility.FiringSolutionHelper;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class DriveManual extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */
  private final Drive drive;

  private final DriveManualStateMachine stateMachine;

  private double driveX = 0;
  private double driveY = 0;
  private double rotatePower = 0;

  private double driveRawX;
  private double driveRawY;
  private double rotateRaw;
  private double driveAngle;
  private double driveAbsAngularVel;

  private double speakerCentricAngle;

  private Double pseudoAutoRotateAngle;

  private Timer spinoutActivationTimer = new Timer();
  private Timer spinoutActivationTimer2 = new Timer();
  private LockedWheel lockedWheelState;
  private double initialSpinoutAngle;

  public enum LockedWheel {
    none,
    center,
    frontLeft,
    backLeft,
    backRight,
    frontRight;
  }

  public DriveManual() {
    drive = Drive.getInstance();
    stateMachine = new DriveManualStateMachine(DriveManualState.DEFAULT);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetRotatePID();

    spinoutActivationTimer.stop();
    spinoutActivationTimer2.stop();
    spinoutActivationTimer.reset();
    spinoutActivationTimer2.reset();
    lockedWheelState = LockedWheel.none;
  }

  @Override
  public void execute() {
    updateDriveValues();
    switch (stateMachine.getState()) {
      case SPEAKER_CENTRIC:
        final int speakerCenterAprilTagID;
        final int speakerSideAprilTagID;
        if (Robot.isRed()) {
          speakerCenterAprilTagID = Constants.FieldConstants.redSpeakerCenterTagID;
          speakerSideAprilTagID = Constants.FieldConstants.redSpeakerSideTagID;
        } else {
          speakerCenterAprilTagID = Constants.FieldConstants.blueSpeakerCenterTagID;
          speakerSideAprilTagID = Constants.FieldConstants.blueSpeakerSideTagID;
        }

        // check for both speaker tags visible
        if (Limelight.getOuttakeInstance().getSpecifiedTagVisible(speakerCenterAprilTagID) 
              && Limelight.getOuttakeInstance().getSpecifiedTagVisible(speakerSideAprilTagID)) {
          final Pose2d botPoseFieldRelative = Limelight.getOuttakeInstance().getBotposeWpiBlue();
          final Translation2d botPoseToSpeaker = FiringSolutionHelper.getVectorToSpeaker(botPoseFieldRelative.getX(), botPoseFieldRelative.getY());
          
          speakerCentricAngle = botPoseToSpeaker.getAngle().getDegrees();
          drive.driveAutoRotate(driveX, driveY, speakerCentricAngle);
          LED.getInstance().setAutoRotateDebugLed(Color.kViolet, Constants.LED.debugLed2);

          Logger.recordOutput("RobotHeading/PseudoAutoRotateEngaged", false);
          Logger.recordOutput("RobotHeading/SpeakerCentricHeading/", speakerCentricAngle);
          Logger.recordOutput("RobotHeading/State", "Speaker centric");
          return;
        }

        // check if we can rotate to see the center tag
        if (Limelight.getOuttakeInstance().getSpecifiedTagVisible(speakerSideAprilTagID)) {
          speakerCentricAngle = Drive.getInstance().getAngle() - 
            Limelight.getOuttakeInstance().getTag(speakerSideAprilTagID).tx;
          drive.driveAutoRotate(driveX, driveY, speakerCentricAngle);
          LED.getInstance().setAutoRotateDebugLed(Color.kDarkViolet, Constants.LED.debugLed2);

          Logger.recordOutput("RobotHeading/PseudoAutoRotateEngaged", false);
          Logger.recordOutput("RobotHeading/SpeakerCentricHeading/", speakerCentricAngle);
          Logger.recordOutput("RobotHeading/State", "Speaker centric search right");
          return;
        }

        // check if we can rotate to see the side tag
        if (Limelight.getOuttakeInstance().getSpecifiedTagVisible(speakerCenterAprilTagID)) {
          speakerCentricAngle = Drive.getInstance().getAngle() - 
            Limelight.getOuttakeInstance().getTag(speakerCenterAprilTagID).tx;
          drive.driveAutoRotate(driveX, driveY, speakerCentricAngle);
          LED.getInstance().setAutoRotateDebugLed(Color.kDarkViolet, Constants.LED.debugLed2);

          Logger.recordOutput("RobotHeading/PseudoAutoRotateEngaged", false);
          Logger.recordOutput("RobotHeading/SpeakerCentricHeading/", speakerCentricAngle);
          Logger.recordOutput("RobotHeading/State", "Speaker centric search left");
          return;
        }

        // If limelight fails to detect an april tag, then run regular drive logic
        break;
      case ROBOT_CENTRIC:
        // Make robot angle zero to switch to robot centric driving
        Logger.recordOutput("RobotHeading/PseudoAutoRotateEngaged", false);
        Logger.recordOutput("RobotHeading/State", "Robot centric");
        LED.getInstance().setAutoRotateDebugLed(Color.kYellow, Constants.LED.debugLed2);
        drive.drive(driveX, driveY, rotatePower, new Rotation2d());
        return;
      case DEFAULT:
        // Run regular drive logic
        break;
    }
    
    // Default to regular field centric drive logic if other drive modes fail to engage
    if (rotatePower != 0) {
      doSpinout();
    } else if (drive.isAutoRotateTuningEnabled()) {
      LED.getInstance().setAutoRotateDebugLed(Color.kPink, Constants.LED.debugLed2);
      drive.driveAutoRotate(driveX, driveY, 0);
    } else if (drive.isPseudoAutoRotateEnabled() && pseudoAutoRotateAngle != null) {
      Logger.recordOutput("RobotHeading/PseudoAutoRotateEngaged", true);
      Logger.recordOutput("RobotHeading/PseudoAutoRotateHeading", pseudoAutoRotateAngle);
      LED.getInstance().setAutoRotateDebugLed(Color.kBlue, Constants.LED.debugLed2);
      drive.driveAutoRotate(driveX, driveY, pseudoAutoRotateAngle);
    } else {
      LED.getInstance().setAutoRotateDebugLed(Color.kBlack, Constants.LED.debugLed2);
      LED.getInstance().setAutoRotateDebugLed(Color.kBlack, Constants.LED.debugLed3);
      LED.getInstance().setAutoRotateDebugLed(Color.kBlack, Constants.LED.debugLed4);
      drive.drive(driveX, driveY, rotatePower);
    }
  }

  public void updateStateMachine(DriveManualTrigger trigger) {
    stateMachine.fire(trigger);
  }

  private void updateDriveValues() {
    final double driveDeadband;
    final double rotateLeftDeadband;
    final double rotateRightDeadband;

    driveAngle = drive.getAngle();
    driveAbsAngularVel = Math.abs(drive.getAngularVelocity());

    // switch between joysticks and xbox which can reconfigure drive stick location
    switch (drive.getControlType()) {
      case Constants.ControllerTypeStrings.joysticks:
        driveRawX = -RobotContainer.driveStick.getY();
        driveRawY = -RobotContainer.driveStick.getX();
        rotateRaw = -RobotContainer.rotateStick.getZ();

        driveDeadband = Manual.joystickDriveDeadband;
        rotateLeftDeadband = Manual.joystickRotateLeftDeadband;
        rotateRightDeadband = Manual.joystickRotateRightDeadband;
        Logger.recordOutput("UnknownControllerType", false);
        break;

      case Constants.ControllerTypeStrings.xboxLeftDrive:
        driveRawX = -RobotContainer.driveXbox.getLeftY();
        driveRawY = -RobotContainer.driveXbox.getLeftX();
        rotateRaw = -RobotContainer.driveXbox.getRightX();

        driveDeadband = Manual.xboxDriveDeadband;
        rotateLeftDeadband = Manual.xboxRotateDeadband;
        rotateRightDeadband = Manual.xboxRotateDeadband;
        Logger.recordOutput("UnknownControllerType", false);
        break;

      case Constants.ControllerTypeStrings.xboxRightDrive:
        driveRawX = -RobotContainer.driveXbox.getRightY();
        driveRawY = -RobotContainer.driveXbox.getRightX();
        rotateRaw = -RobotContainer.driveXbox.getLeftX();

        driveDeadband = Manual.xboxDriveDeadband;
        rotateLeftDeadband = Manual.xboxRotateDeadband;
        rotateRightDeadband = Manual.xboxRotateDeadband;
        Logger.recordOutput("UnknownControllerType", false);
        break;

      default:
        Logger.recordOutput("UnknownControllerType", true);
        return;
    }

    // Joystick polarity:
    // Positive X is to the right
    // Positive Y is down
    // Positive Z is CW

    // WPI uses a trigonometric coordinate system with the front of the robot
    // pointing toward positive X. Thus:
    // Positive X is forward
    // Positive Y is to the left
    // Positive angles are CCW
    // Angles have a range of +/- 180 degrees (need to verify this)

    // All variables in this program use WPI coordinates
    // All "theta" variables are in radians

    // Convert raw drive inputs to polar coordinates for more precise deadband
    // correction
    final double driveRawMag = OrangeMath.pythag(driveRawX, driveRawY);
    final double driveRawTheta = Math.atan2(driveRawY, driveRawX);

    // Normalize the drive input over deadband in polar coordinates.
    double driveMag = 0;
    if (driveRawMag > driveDeadband) {
      driveMag = (driveRawMag - driveDeadband) / (1 - driveDeadband);

      if (Constants.driveTuningMode) {
        // quantize input drive magnitude to 0, 0.25, 0.5, 0.75, 1.0 for PID tuning
        driveMag = Math.round(driveMag * 4.0) / 4.0;
      }

      switch (drive.getDriveInputScaling()) {
        case DriveInputScalingStrings.linear:
          break;
        case DriveInputScalingStrings.quadratic:
          driveMag = driveMag * driveMag;
          break;
        case DriveInputScalingStrings.cubic:
          driveMag = driveMag * driveMag * driveMag;
          break;
      }

      // Normalize vector magnitude so as not to give an invalid input
      if (driveMag > 1) {
        driveMag = 1;
      }
    }
    // Convert back to cartesian coordinates
    driveX = Math.cos(driveRawTheta) * driveMag;
    driveY = Math.sin(driveRawTheta) * driveMag;

    // Normalize the rotation inputs over deadband.
    rotatePower = 0;
    if (rotateRaw > rotateLeftDeadband) {
      rotatePower = (rotateRaw - rotateLeftDeadband) / (1 - rotateLeftDeadband);
    } else if (rotateRaw < -rotateRightDeadband) {
      rotatePower = (rotateRaw + rotateRightDeadband) / (1 - rotateRightDeadband);
    }

    switch (drive.getRotateInputScaling()) {
      case RotateInputScalingStrings.linear:
        break;
      case RotateInputScalingStrings.squareRoot:
        rotatePower = Math.signum(rotatePower) * Math.sqrt(Math.abs(rotatePower));
        break;
      case RotateInputScalingStrings.quadratic:
        rotatePower = Math.signum(rotatePower) * (rotatePower * rotatePower);
        break;
      case RotateInputScalingStrings.power:
        rotatePower =
            Math.signum(rotatePower)
                * Math.pow(Math.abs(rotatePower), drive.getRotationPowerScaling());
        break;
    }
    rotatePower = rotatePower * drive.getMaxManualRotationEntry();

    if (stateMachine.getState() != DriveManualState.DEFAULT || rotatePower != 0) {
      pseudoAutoRotateAngle = null;
      LED.getInstance().setAutoRotateDebugLed(Color.kBlack, Constants.LED.debugLed1);
    } else if (rotatePower == 0
        && pseudoAutoRotateAngle == null
        && driveAbsAngularVel < Manual.inhibitPseudoAutoRotateDegPerSec) {
      pseudoAutoRotateAngle = drive.getAngle();
      LED.getInstance().setAutoRotateDebugLed(Color.kBlue, Constants.LED.debugLed1);
    }
  }

  private void doSpinout() {
    if (Math.abs(rotateRaw) >= Manual.spinoutRotateDeadBand) {
      if (driveAbsAngularVel < Manual.spinoutMinAngularVelocity) {
        spinoutActivationTimer.start();
      } else {
        spinoutActivationTimer.stop();
        spinoutActivationTimer.reset();
      }
      if (driveAbsAngularVel < Manual.spinoutMinAngularVelocity2) {
        spinoutActivationTimer2.start();
      } else {
        spinoutActivationTimer2.stop();
        spinoutActivationTimer2.reset();
      }
    } else {
      // if rotation stick falls under second deadband, reset rotation back to normal
      lockedWheelState = LockedWheel.none;
      spinoutActivationTimer.stop();
      spinoutActivationTimer2.stop();
      spinoutActivationTimer.reset();
      spinoutActivationTimer2.reset();
    }

    // detect if not rotating and if rotate stick past second deadband for certain
    // amount of time
    // (first deadband is rotateToleranceDegrees/xboxRotateDeadband)
    // (second deadband is past first deadband in rotation) (close to max rotation)
    if ((lockedWheelState == LockedWheel.none)
        && (spinoutActivationTimer.hasElapsed(Manual.spinoutActivationSec)
            || spinoutActivationTimer2.hasElapsed(Manual.spinout2ActivationSec))) {

      // from this, figure out which swerve module to lock onto to rotate off of (use
      // drive
      // stick direction and robotAngle)
      // How to use drive stick: module closest to direction of drivestick.
      // use gyro to find orientation
      // algorithm to determine quadrant: driveStickAngle - robotAngle
      // if drivestick angle 0 < x < 90 , in quadrant 1 (front left module)
      // if drivestick angle 90 < x < 180 , in quadrant 2 (back left module)
      // if drivestick angle -180 < x < -90 , in quadrant 3 (back right module)
      // if drivestick angle -90 < x < 0 , in quadrant 4 (front right module)

      // SPECIAL CASE: if driveStickAngle - robotAngle is exactly 0, 90, 180, -180,
      // then use the
      // rotate angle to determine wheel:
      // 0: if CW, quadrant 1 (front left); if CCW, quadrant 4 (front right)
      // 90: if CW, quadrant 2 (back left); if CCW, quadrant 1 (front left)
      // 180/-180: if CW, quadrant 3 (back right); if CCW, quadrant 2 (back left)
      // -90: if CW, quadrant 4 (front right); if CCW, quadrant 3 (back right)

      // drivestick angle - robot angle
      double robotCentricDriveTheta =
          OrangeMath.boundDegrees(Math.toDegrees(Math.atan2(driveY, driveX)) - driveAngle);
      initialSpinoutAngle = driveAngle;

      if (Constants.spinoutCenterEnabled && (driveX == 0) && (driveY == 0)) {
        lockedWheelState = LockedWheel.center;
      } else if (Constants.spinoutCornerEnabled) {
        if ((robotCentricDriveTheta > 0) && (robotCentricDriveTheta < 90)) {
          lockedWheelState = LockedWheel.frontLeft;
        } else if ((robotCentricDriveTheta > 90) && (robotCentricDriveTheta < 180)) {
          lockedWheelState = LockedWheel.backLeft;
        } else if ((robotCentricDriveTheta > -180) && (robotCentricDriveTheta < -90)) {
          lockedWheelState = LockedWheel.backRight;
        } else if ((robotCentricDriveTheta > -90) && (robotCentricDriveTheta < 0)) {
          lockedWheelState = LockedWheel.frontRight;
        }
      }

      // if robot rotates 90 degrees, reset rotation back to normal
    } else if ((Math.abs(initialSpinoutAngle - driveAngle) >= 90)
        && (lockedWheelState != LockedWheel.none)) {
      lockedWheelState = LockedWheel.none;
      spinoutActivationTimer.stop();
      spinoutActivationTimer2.stop();
      spinoutActivationTimer.reset();
      spinoutActivationTimer2.reset();
    }
    // use state machine for rotating each wheel in each direction (8 cases)
    // each module rotating CW and CCW
    double spinCornerPower = Math.copySign(DriveConstants.spinoutCornerPower, rotatePower);
    switch (lockedWheelState) {
      case none:
        LED.getInstance().setAutoRotateDebugLed(Color.kBlack, Constants.LED.debugLed2);
        drive.drive(driveX, driveY, rotatePower);
        break;
      case center:
        LED.getInstance().setAutoRotateDebugLed(Color.kWhite, Constants.LED.debugLed2); // orange
        drive.drive(driveX, driveY, Math.copySign(DriveConstants.spinoutCenterPower, rotatePower));
        break;
      case frontLeft:
        LED.getInstance().setAutoRotateDebugLed(Color.kOrange, Constants.LED.debugLed2); // orange
        drive.drive(driveX, driveY, spinCornerPower, DriveConstants.frontLeftWheelLocation);
        break;
      case backLeft:
        LED.getInstance().setAutoRotateDebugLed(Color.kOrange, Constants.LED.debugLed2);
        drive.drive(driveX, driveY, spinCornerPower, DriveConstants.backLeftWheelLocation);
        break;
      case backRight:
        LED.getInstance().setAutoRotateDebugLed(Color.kOrange, Constants.LED.debugLed2);
        drive.drive(driveX, driveY, spinCornerPower, DriveConstants.backRightWheelLocation);
        break;
      case frontRight:
        LED.getInstance().setAutoRotateDebugLed(Color.kOrange, Constants.LED.debugLed2);
        drive.drive(driveX, driveY, spinCornerPower, DriveConstants.frontRightWheelLocation);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      pseudoAutoRotateAngle = null;
      LED.getInstance().setAutoRotateDebugLed(Color.kBlack, Constants.LED.debugLed1);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
