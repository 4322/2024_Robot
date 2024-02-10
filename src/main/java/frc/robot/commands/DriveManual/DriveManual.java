package frc.robot.commands.DriveManual;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.Manual;
import frc.robot.Constants.InputScalingStrings;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualState;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualTrigger;
import frc.robot.subsystems.drive.DriveInterface;
import frc.utility.OrangeMath;
import frc.utility.PositionVector;
import org.littletonrobotics.junction.Logger;

public class DriveManual extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */
  private final DriveInterface drive;

  private final DriveManualStateMachine stateMachine;

  private double driveX = 0;
  private double driveY = 0;
  private double rotatePower = 0;

  public DriveManual(DriveInterface drivesubsystem) {
    drive = drivesubsystem;
    stateMachine = new DriveManualStateMachine(DriveManualState.DEFAULT);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetRotatePID();
  }

  @Override
  public void execute() {
    updateDriveValues();

    switch (stateMachine.getState()) {
      case DEFAULT:
        if (drive.isPseudoAutoRotateEnabled()
            && Math.abs(drive.getAngularVelocity())
                < Manual.inhibitPseudoAutoRotateAngularVelocity) {
          drive.driveAutoRotate(driveX, driveY, drive.getAngle());
        } else {
          drive.drive(driveX, driveY, rotatePower);
        }
        break;
      case SPEAKER_CENTRIC:
        Pose2d drivePose2D = drive.getPose2d();
        Translation2d speakerVec =
            PositionVector.getVectorToSpeaker(drivePose2D.getX(), drivePose2D.getY());
        Logger.recordOutput("SpeakerCentricHeading", speakerVec.getAngle().getDegrees());
        drive.driveAutoRotate(driveX, driveY, speakerVec.getAngle().getDegrees());
    }
  }

  public void updateStateMachine(DriveManualTrigger trigger) {
    stateMachine.fire(trigger);
  }

  private void updateDriveValues() {
    final double driveRawX;
    final double driveRawY;
    final double rotateRaw;

    final double driveDeadband;
    final double rotateLeftDeadband;
    final double rotateRightDeadband;

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
        driveRawX = -RobotContainer.xbox.getLeftY();
        driveRawY = -RobotContainer.xbox.getLeftX();
        rotateRaw = -RobotContainer.xbox.getRightX();

        driveDeadband = Manual.xboxDriveDeadband;
        rotateLeftDeadband = Manual.xboxRotateDeadband;
        rotateRightDeadband = Manual.xboxRotateDeadband;
        Logger.recordOutput("UnknownControllerType", false);
        break;

      case Constants.ControllerTypeStrings.xboxRightDrive:
        driveRawX = -RobotContainer.xbox.getRightY();
        driveRawY = -RobotContainer.xbox.getRightX();
        rotateRaw = -RobotContainer.xbox.getLeftX();

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
    driveX = Math.cos(driveRawTheta) * driveMag;
    driveY = Math.sin(driveRawTheta) * driveMag;

    // Normalize the rotation inputs over deadband.
    rotatePower = 0;
    if (rotateRaw > rotateLeftDeadband) {
      rotatePower = (rotateRaw - rotateLeftDeadband) / (1 - rotateLeftDeadband);
    } else if (rotateRaw < -rotateRightDeadband) {
      rotatePower = (rotateRaw + rotateRightDeadband) / (1 - rotateRightDeadband);
    }
    rotatePower = rotatePower * drive.getMaxManualRotationEntry();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
