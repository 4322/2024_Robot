package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.noteTracker.NoteTracker;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.FiringSolutionHelper;
import frc.utility.OrangeMath;

public class RobotCoordinator extends SubsystemBase {
  private Intake intake = Intake.getInstance();
  private Outtake outtake = Outtake.getInstance();
  private Drive drive = Drive.getInstance();
  private NoteTracker noteTracker = NoteTracker.getInstance();
  private Limelight intakeLimelight = Limelight.getIntakeInstance();

  private static RobotCoordinator robotCoordinator;
  private boolean intakeButtonPressed;
  private boolean autoIntakeButtonPressed;

  public static RobotCoordinator getInstance() {
    if (robotCoordinator == null) {
      robotCoordinator = new RobotCoordinator();
    }
    return robotCoordinator;
  }

  private RobotCoordinator() {}

  // *****BELOW METHODS PERTAIN TO PHYSICAL LIMITATIONS OF ROBOT / SAFETY CHECKS*****
  public boolean canDeploy() {
    return !intake.isDeployed();
  }

  public boolean canRetract() {
    return !intake.isFeeding() && !intake.isEjecting();
  }

  public boolean canShoot() {
    return outtake.isFlyWheelUpToSpeed() && outtake.pivotIsAtPosition();
  }

  public boolean canSmartShoot() {
    return canShoot() && alignedWithSpeaker();
  }

  public boolean canSpinFlywheel() {
    return Outtake.getInstance().pivotIsInitialized();
  }

  public boolean canPivot() {
    return outtake.pivotIsInitialized();
  }

  // *****METHODS BELOW PERTAIN TO SUBSYTEM STATE INFORMATION*****
  public void setIntakeButtonState(boolean isPressed) {
    intakeButtonPressed = isPressed;
  }

  public boolean inShotTuningMode() {
    return outtake.inShotTuningMode();
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

  public boolean isInitialized() {
    return outtake.pivotIsInitialized();
  }

  public boolean intakeIsFeeding() {
    return intake.isFeeding();
  }

  public boolean intakeIsEjecting() {
    return intake.isEjecting();
  }

  public boolean outtakeIsFeeding() {
    return outtake.isFeeding();
  }

  public boolean outtakeIsEjecting() {
    return outtake.isOuttaking();
  }

  public boolean noteInFiringPosition() {
    return noteTracker.tunnelBeamBroken();
  }

  public boolean noteEnteringIntake() {
    return noteTracker.intakeBeamBroken() && intake.isFeeding();
  }

  public boolean noteIsShot() {
    return noteTracker.noteIsShot();
  }

  public boolean noteEjectingThroughIntake() {
    return noteTracker.intakeBeamBroken() && intake.isEjecting();
  }

  public boolean noteInRobot() {
    return noteTracker.intakeBeamBroken()
        || noteTracker.tunnelBeamBroken()
        || noteTracker.notePassingIntake()
        || noteTracker.notePassingTunnel();
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

  public boolean alignedWithSpeaker() {
    final int centerTagID;
    final int sideTagID;
    if (Robot.isRed()) {
      centerTagID = Constants.FieldConstants.redSpeakerCenterTagID;
      sideTagID = Constants.FieldConstants.redSpeakerSideTagID;
    }
    else {
      centerTagID = Constants.FieldConstants.blueSpeakerCenterTagID;
      sideTagID = Constants.FieldConstants.blueSpeakerSideTagID;
    }
    if (Limelight.getOuttakeInstance().getSpecifiedTagVisible(centerTagID)
          && Limelight.getOuttakeInstance().getSpecifiedTagVisible(sideTagID)) {
        final Pose2d robotPoseFieldRelative = Limelight.getOuttakeInstance().getBotposeWpiBlue();
        final Translation2d botPoseToSpeaker = 
          FiringSolutionHelper.getVectorToSpeaker(robotPoseFieldRelative.getX(), robotPoseFieldRelative.getY());

        // Must bound botPoseToSpeaker angle because getAnge drive method is bounded
        return OrangeMath.equalToEpsilon(OrangeMath.boundDegrees(botPoseToSpeaker.getAngle().getDegrees()), 
            drive.getAngle(), Constants.LimelightConstants.alignToSpeakerTagRotTolerance);
      }
      return false;
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

  public boolean debugOuttakeOverride() {
    return outtake.getDebugOverrideEnabled();
  }

  public boolean deployInCoast() {
    return intake.deployInCoast();
  }

  public boolean pivotInCoast() {
    return outtake.pivotInCoast();
  }

  public boolean outtakeFlyWheelUpTospeed() {
    return outtake.isFlyWheelUpToSpeed();
  }
}
