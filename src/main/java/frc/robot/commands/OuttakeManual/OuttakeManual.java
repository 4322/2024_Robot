package frc.robot.commands.OuttakeManual;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.utility.FiringSolutionHelper;

import org.littletonrobotics.junction.Logger;

public class OuttakeManual extends Command {
  private final Outtake outtake;
  private final Limelight outtakeLimelight;
  private FiringSolution firingSolution;
  private double smartShootingOffset = Constants.OuttakeConstants.pivotSmartShootingOffset;

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
    if (!RobotCoordinator.getInstance().inShotTuningMode()) {
      switch (stateMachine.getState()) {
        case SMART_SHOOTING:
          final int speakerCenterAprilTagID;
          final int speakerSideAprilTagID;
          if (Robot.isRed()) {
            speakerCenterAprilTagID = Constants.FieldConstants.redSpeakerCenterTagID;
            speakerSideAprilTagID = Constants.FieldConstants.redSpeakerSideTagID;
          }
          else {
            speakerCenterAprilTagID = Constants.FieldConstants.blueSpeakerCenterTagID;
            speakerSideAprilTagID = Constants.FieldConstants.blueSpeakerSideTagID;
          }

          // Check if the apriltag is physically connected
          if (outtakeLimelight.isNetworkTableConnected()) {
            // Only calculate new firing solutions if we can see the target April tags with Limelight.
            // If specificed targets aren't visible, then shooter stays at the previous calculated or set solution.
            if (outtakeLimelight.getSpecifiedTagVisible(speakerCenterAprilTagID)
                  && outtakeLimelight.getSpecifiedTagVisible(speakerSideAprilTagID)) {
              final Pose2d botPoseFieldRelative = outtakeLimelight.getBotposeWpiBlue();
              final Translation2d botPoseToSpeaker = FiringSolutionHelper.getVectorToSpeaker(botPoseFieldRelative.getX(), botPoseFieldRelative.getY());
              
              double magToSpeaker = botPoseToSpeaker.getNorm();
              double degreesToSpeaker = botPoseToSpeaker.getAngle().getDegrees();
              firingSolution = FiringSolutionManager.getInstance().calcSolution(magToSpeaker, degreesToSpeaker);

              double adjShotRotations = firingSolution.getShotRotations() + smartShootingOffset;

              firingSolution = new FiringSolution(0, 0, firingSolution.getFlywheelSpeed(), adjShotRotations);
              
              Logger.recordOutput("FiringSolutions/BotPoseInput/Mag", magToSpeaker);
              Logger.recordOutput("FiringSolutions/BotPoseInput/Angle", degreesToSpeaker);
            }
            else {
              Logger.recordOutput("FiringSolutions/BotPoseInput/Mag", 0.0);
              Logger.recordOutput("FiringSolutions/BotPoseInput/Angle", 0.0);
            }
            
            Logger.recordOutput("FiringSolutions/CalculatedShot/Flywheel", firingSolution.getFlywheelSpeed());
            Logger.recordOutput("FiringSolutions/CalculatedShot/PivotRotations", firingSolution.getShotRotations());
          } else {
            Logger.recordOutput("FiringSolutions/BotPoseInput/Mag", 0.0);
            Logger.recordOutput("FiringSolutions/BotPoseInput/Angle", 0.0);
          }
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
        case STARTING_CONFIG:
          firingSolution = FiringSolutions.StartingConfig;
          break;
        case PASS:
          firingSolution = FiringSolutions.Pass;
          break;
        case AMP:
          outtake.pivot(Constants.OuttakeConstants.ampPivotRotations);
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
        outtake.pivot(firingSolution.getShotRotations());
      } else {
        outtake.stopPivot();
      }
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

  public void addOffset(double offsetValue) {
    smartShootingOffset += offsetValue;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
