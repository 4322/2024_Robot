package frc.robot.commands.OuttakeManual;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FiringSolutions;
import frc.robot.Robot;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualState;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualTrigger;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.outtake.Outtake;

import org.littletonrobotics.junction.Logger;

public class OuttakeManual extends Command {
  private final Outtake outtake;
  private final Limelight outtakeLimelight;
  private FiringSolution firingSolution;

  private static final OuttakeManualStateMachine stateMachine =
      new OuttakeManualStateMachine(OuttakeManualState.STOP);

  public OuttakeManual() {
    outtake = Outtake.getInstance();
    outtakeLimelight = Limelight.getOuttakeInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch (stateMachine.getState()) {
      case SMART_SHOOTING:
        final int speakerAprilTagID;
        if (Robot.isRed()) {
          speakerAprilTagID = Constants.FieldConstants.redSpeakerCenterTagID;
        }
        else {
          speakerAprilTagID = Constants.FieldConstants.blueSpeakerCenterTagID;
        }

        // Only calculate new firing solutions if we can see the target April tag with Limelight.
        // If specificed target isn't visible, then shooter stays at the previous calculated or set solution.
        if (outtakeLimelight.getSpecifiedAprilTagVisible(speakerAprilTagID)) {
          final Pose2d botPoseToSpeaker = outtakeLimelight.getTargetPose3DToBot(Constants.FieldConstants.redSpeakerCenterTagID).toPose2d();
          
          double magToSpeaker = botPoseToSpeaker.getTranslation().getNorm();
          double degreesToSpeaker = botPoseToSpeaker.getRotation().getDegrees();
          firingSolution = FiringSolutionManager.getInstance().calcSolution(magToSpeaker, degreesToSpeaker);
          
          Logger.recordOutput("FiringSolutions/BotPoseInput/Mag", magToSpeaker);
          Logger.recordOutput("FiringSolutions/BotPoseInput/Angle", degreesToSpeaker);
        }
        
        Logger.recordOutput("FiringSolutions/CalculatedShot", firingSolution.toString());
        break;
      case SUBWOOFER:
        firingSolution = FiringSolutions.SubwooferBase;
        break;
      case EJECT:
        firingSolution = FiringSolutions.Eject;
        break;
      case COLLECTING_NOTE:
        firingSolution = FiringSolutions.CollectingNote;
        // lockout of presets until the note is safely in the outtake
        // change to stopped state when note triggers the tunnel sensor
        if (RobotCoordinator.getInstance().noteInFiringPosition()) {
          updateStateMachine(OuttakeManualTrigger.ENABLE_STOP);
        }
        break;
      case FEED:
        firingSolution = FiringSolutions.Feed;
        break;
      case CLIMBING:
        firingSolution = FiringSolutions.Climbing;
        break;
      case AMP:
        outtake.outtake(Constants.OuttakeConstants.ampTopShooterRPS, Constants.OuttakeConstants.ampBottomShooterRPS); 
        return;
      case STOP:
      default:
        outtake.stopOuttake();
        outtake.stopPivot();
        return;
    }

    if (RobotCoordinator.getInstance().canSpinFlywheel()) {
      outtake.outtake(firingSolution.getFlywheelSpeed());
    } else {
      outtake.stopOuttake();
    }

    if (RobotCoordinator.getInstance().canPivot()) {
      if (stateMachine.getState() == OuttakeManualState.CLIMBING) {
        outtake.pivot(firingSolution.getShotRotations(), false);
      } else {
        outtake.pivot(firingSolution.getShotRotations(), true);
      }
    } else {
      outtake.stopPivot();
    }
  }

  public static OuttakeManualState getState() {
    return stateMachine.getState();
  }

  public void updateStateMachine(OuttakeManualTrigger trigger) {
    stateMachine.fire(trigger);
  }

  public void setFiringSolution(FiringSolution solution) {
    firingSolution = solution;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
