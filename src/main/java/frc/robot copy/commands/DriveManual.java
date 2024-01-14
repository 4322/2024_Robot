package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.utility.OrangeMath;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputScalingStrings;
import frc.robot.Constants.DriveConstants.Manual;
import frc.robot.RobotContainer;

public class DriveManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private static boolean scoreAutoPoseActive;
  private static boolean loadAutoPoseActive;
  private static boolean loadAutoAlignPending;
  private static boolean armAtLoadSingle;
  private final Drive drive;
  private final AutoPose autoPose;
  private Double targetHeadingDeg;
  private boolean done;
  private Timer spinoutActivationTimer = new Timer();
  private Timer spinoutActivationTimer2 = new Timer();
  private LockedWheel lockedWheelState;
  private double initialSpinoutAngle;

  public enum AutoPose {
    none, usePreset, usePresetAuto, usePresetManual, usePresetNoArmMove, loadSingleManual
  }
  
  public enum LockedWheel {
    none, center, frontLeft, backLeft, backRight, frontRight;
  }

  public static boolean isScoreAutoPoseActive() {
    return scoreAutoPoseActive;
  }

  public static boolean isLoadAutoPoseActive() {
    return loadAutoPoseActive;
  }

  public static boolean isLoadAutoAlignReady() {
    return loadAutoAlignPending && armAtLoadSingle;
  }

  // prevent AutoAlignSubstation command from immediately restarting once complete
  public static void loadAutoAlignDone() {
    loadAutoAlignPending = false;
  }

  public DriveManual(Drive drivesubsystem, AutoPose autoPose) {
    drive = drivesubsystem;
    this.autoPose = autoPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // make command reusable
    spinoutActivationTimer.stop();
    spinoutActivationTimer2.stop();
    spinoutActivationTimer.reset();
    spinoutActivationTimer2.reset();
    lockedWheelState = LockedWheel.none;
    done = false;

    drive.resetRotatePID();
    scoreAutoPoseActive = false;
    loadAutoPoseActive = false;
    loadAutoAlignPending = false;
    armAtLoadSingle = false;

    // set auto-rotate direction, if any
    switch (autoPose) {
      case none:
        targetHeadingDeg = null;
        break;
      case loadSingleManual:
        break;
      case usePresetAuto:
        // fall through
      case usePreset:
        autoRotateSetTarget(true);
        break;
      case usePresetManual:
      case usePresetNoArmMove:
        autoRotateSetTarget(false);
        break;
    }
  }

  // autoAlignMode is set to true when we want to invoke autoAlignment
  private void autoRotateSetTarget(boolean autoAlignMode) {
    targetHeadingDeg = 0.0;
  }

  @Override
  public void execute() {

    final double driveRawX;
    final double driveRawY;
    final double rotateRaw;

    final double driveDeadband;
    final double rotateLeftDeadband;
    final double rotateRightDeadband;

    //switch between joysticks and xbox which can reconfigure drive stick location
    switch(drive.getControlType()) {
      case Constants.ControllerTypeStrings.joysticks:
        driveRawX = -RobotContainer.driveStick.getY();
        driveRawY = -RobotContainer.driveStick.getX();
        rotateRaw = -RobotContainer.rotateStick.getZ();

        driveDeadband = Manual.joystickDriveDeadband;
        rotateLeftDeadband = Manual.joystickRotateLeftDeadband;
        rotateRightDeadband = Manual.joystickRotateRightDeadband;
        Logger.getInstance().recordOutput("UnknownControllerType", false);
        break;

      case Constants.ControllerTypeStrings.xboxLeftDrive:
        driveRawX = -RobotContainer.xbox.getLeftY();
        driveRawY = -RobotContainer.xbox.getLeftX();
        rotateRaw = -RobotContainer.xbox.getRightX();

        driveDeadband = Manual.xboxDriveDeadband;
        rotateLeftDeadband = Manual.xboxRotateDeadband;
        rotateRightDeadband = Manual.xboxRotateDeadband;
        Logger.getInstance().recordOutput("UnknownControllerType", false);
        break;
      
      case Constants.ControllerTypeStrings.xboxRightDrive:
        driveRawX = -RobotContainer.xbox.getRightY();
        driveRawY = -RobotContainer.xbox.getRightX();
        rotateRaw = -RobotContainer.xbox.getLeftX();

        driveDeadband = Manual.xboxDriveDeadband;
        rotateLeftDeadband = Manual.xboxRotateDeadband;
        rotateRightDeadband = Manual.xboxRotateDeadband;
        Logger.getInstance().recordOutput("UnknownControllerType", false);
        break;

      default:
        Logger.getInstance().recordOutput("UnknownControllerType", true);
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

      // Dual driver inputs need to be processed in an additive manner
      // instead of being averaged to avoid discontinuities.

      // Cache hardware status for consistency in logic and convert
      // joystick/Xbox coordinates to WPI coordinates.

      // Convert raw drive inputs to polar coordinates for more precise deadband
      // correction
      final double driveRawMag = OrangeMath.pythag(driveRawX, driveRawY);
      final double driveRawTheta = Math.atan2(driveRawY, driveRawX);

      final double driveAngle = drive.getAngle();

      // Normalize the drive input over deadband in polar coordinates.
      double driveMag = 0;
      if (driveRawMag > driveDeadband) {
        driveMag = (driveRawMag - driveDeadband) / (1 - driveDeadband);

        if (Constants.driveTuningMode) {
          // quantize input drive magnitude to 0, 0.25, 0.5, 0.75, 1.0 for PID tuning
          driveMag = Math.round(driveMag * 4.0) / 4.0;
        }

        switch (drive.getInputScaling()) {
          case InputScalingStrings.linear:
            break;
          case InputScalingStrings.quadratic:
            driveMag = driveMag * driveMag;
            break;
          case InputScalingStrings.cubic:
            driveMag = driveMag * driveMag * driveMag;
            break;
        }

        // Normalize vector magnitude so as not to give an invalid input
        if (driveMag > 1) {
          driveMag = 1;
        }
      }
      // Convert back to cartesian coordinates
      double driveX = Math.cos(driveRawTheta) * driveMag;
      double driveY = Math.sin(driveRawTheta) * driveMag;

      // Normalize the rotation inputs over deadband.
      double rotatePower = 0;
      if (rotateRaw > rotateLeftDeadband) {
        rotatePower = (rotateRaw - rotateLeftDeadband) / (1 - rotateLeftDeadband);
      } else if (rotateRaw < -rotateRightDeadband) {
        rotatePower = (rotateRaw + rotateRightDeadband) / (1 - rotateRightDeadband);
      }
      rotatePower = rotatePower * drive.getMaxManualRotationEntry();

      // if the rotate stick isn't being used
      if (rotatePower == 0) {
        // if there is a set drive auto rotate
        if (targetHeadingDeg != null) {
          drive.driveAutoRotate(driveX, driveY, targetHeadingDeg);
          return;
        } else if (drive.isPseudoAutoRotateEnabled() && 
            Math.abs(drive.getAngularVelocity()) < Manual.inhibitPseudoAutoRotateAngularVelocity) {
          // set pseudo auto rotate heading
          targetHeadingDeg = driveAngle;
          drive.driveAutoRotate(driveX, driveY, targetHeadingDeg);
          return;
        }
      } else {  
        // rotate joystick is active
        // check if we are in the default drive manual
        if (autoPose == AutoPose.none) {
          targetHeadingDeg = null; // unlock auto rotate heading
        } else {
          // restart default driveManual command
          drive.drive(driveX, driveY, rotatePower);
          done = true;
          return;
        }
      }

      // cache value for logic consistency
      double absAngularVelocity = Math.abs(drive.getAngularVelocity());

      if (Math.abs(rotateRaw) >= Manual.spinoutRotateDeadBand) {
        if (absAngularVelocity < Manual.spinoutMinAngularVelocity) {
          spinoutActivationTimer.start();
        } else {
          spinoutActivationTimer.stop();
          spinoutActivationTimer.reset();
        }
        if (absAngularVelocity < Manual.spinoutMinAngularVelocity2) {
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
          drive.drive(driveX, driveY, rotatePower);
          break;
        case center:
          drive.drive(driveX, driveY, Math.copySign(DriveConstants.spinoutCenterPower, rotatePower));
          break;
        case frontLeft:
          drive.drive(driveX, driveY, spinCornerPower,
              DriveConstants.frontLeftWheelLocation);
          break;
        case backLeft:
          drive.drive(driveX, driveY, spinCornerPower,
              DriveConstants.backLeftWheelLocation);
          break;
        case backRight:
          drive.drive(driveX, driveY, spinCornerPower,
              DriveConstants.backRightWheelLocation);
          break;
        case frontRight:
          drive.drive(driveX, driveY, spinCornerPower,
              DriveConstants.frontRightWheelLocation);
          break;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
