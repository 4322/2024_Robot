package frc.robot.commands.OuttakeManual;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
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

  private Timer smartShootingLockInTimer = new Timer();
  private boolean targetWasVisible;
  private FiringSolution previousSolution;

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
    final FiringSolution solution;
    switch (stateMachine.getState()) {
      case SMART_SHOOTING:
        // Only calculate new firing solutions if we can see the target April tag with Limelight
        if (outtakeLimelight.getTargetVisible()) {
          targetWasVisible = true;
          smartShootingLockInTimer.stop();
          smartShootingLockInTimer.reset();

          final Pose2d botPoseToSpeaker;
          if (Robot.isRed()) {
            botPoseToSpeaker = outtakeLimelight.getTargetPose3DToBot(Constants.FieldConstants.redSpeakerCenterTagID).toPose2d();
          }
          else {
            botPoseToSpeaker = outtakeLimelight.getTargetPose3DToBot(Constants.FieldConstants.blueSpeakerCenterTagID).toPose2d();
          }
          
          // Round to two decimals to reduce noise and have a stable shooting angle and flywheel speed
          double magToSpeaker = Math.round(botPoseToSpeaker.getTranslation().getNorm() * 100.0) / 100.0;
          double degreesToSpeaker = Math.round(botPoseToSpeaker.getRotation().getDegrees() * 100.0) / 100.0;
          solution = FiringSolutionManager.getInstance().calcSolution(magToSpeaker, degreesToSpeaker);
          
          // Save previous solution in case there is a momentary loss of april tag detection and we want 
          // to still keep outtake at same position.
          previousSolution = solution;
          Logger.recordOutput("FiringSolutions/BotPoseInput/Mag", magToSpeaker);
          Logger.recordOutput("FiringSolutions/BotPoseInput/Angle", degreesToSpeaker);
        }
        // Maintain previous firing solution for very short duration to account for momentary loss 
        // of April Tag detection.
        else if (targetWasVisible && !smartShootingLockInTimer.hasElapsed(Constants.LimelightConstants.aprilTagLossThresholdSec)) {
          smartShootingLockInTimer.start();
          solution = previousSolution;
        }
        // If the shooter lock in timer is past the short duration threshold, then reset the smart 
        // shooting position to the default position in preparation for the next time an April tag is detected.
        else {
          smartShootingLockInTimer.stop();
          smartShootingLockInTimer.reset();
          targetWasVisible = false;
          // Optimization where we prepare the shooter to go to the right 
          // position and flywheel speed when no april tag is detected.
          solution = Constants.FiringSolutions.DefaultSmartShooting;

          Logger.recordOutput("FiringSolutions/BotPoseInput/Mag", 0.0);
          Logger.recordOutput("FiringSolutions/BotPoseInput/Angle", 0.0);
        }
        
        Logger.recordOutput("FiringSolutions/CalculatedShot", solution.toString());
        break;
      case SUBWOOFER:
        solution = FiringSolutions.SubwooferBase;
        break;
      case EJECT:
        solution = FiringSolutions.Eject;
        break;
      case COLLECTING_NOTE:
        solution = FiringSolutions.CollectingNote;
        // lockout of presets until the note is safely in the outtake
        // change to stopped state when note triggers the tunnel sensor
        if (RobotCoordinator.getInstance().noteInFiringPosition()) {
          updateStateMachine(OuttakeManualTrigger.ENABLE_STOP);
        }
        break;
      case FEED:
        solution = FiringSolutions.Feed;
        break;
      case CLIMBING:
        solution = FiringSolutions.Climbing;
        break;
      case STOP:
      default:
        outtake.stopOuttake();
        outtake.stopPivot();
        return;
    }

    if (RobotCoordinator.getInstance().canSpinFlywheel()) {
      outtake.outtake(solution.getFlywheelSpeed());
    } else {
      outtake.stopOuttake();
    }

    if (RobotCoordinator.getInstance().canPivot()) {
      if (stateMachine.getState() == OuttakeManualState.CLIMBING) {
        outtake.pivot(solution.getShotRotations(), false);
      } else {
        outtake.pivot(solution.getShotRotations(), true);
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

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
