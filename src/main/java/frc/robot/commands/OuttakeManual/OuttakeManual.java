package frc.robot.commands.OuttakeManual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FiringSolutions;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualState;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualTrigger;
import frc.robot.commands.XboxControllerRumble;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.FiringSolutionHelper;
import org.littletonrobotics.junction.Logger;

public class OuttakeManual extends Command {
  private final Outtake outtake;

  private static final OuttakeManualStateMachine stateMachine =
      new OuttakeManualStateMachine(OuttakeManualState.STOP);

  public OuttakeManual() {
    outtake = Outtake.getInstance();
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
        double botMagToSpeaker =
            FiringSolutionHelper.getVectorToSpeaker(
                    RobotCoordinator.getInstance().getRobotXPos(),
                    RobotCoordinator.getInstance().getRobotYPos())
                .getNorm();
        double botAngleToSpeaker =
            FiringSolutionHelper.getVectorToSpeaker(
                    RobotCoordinator.getInstance().getRobotXPos(),
                    RobotCoordinator.getInstance().getRobotYPos())
                .getAngle()
                .getDegrees();
        solution =
            FiringSolutionManager.getInstance().calcSolution(botMagToSpeaker, botAngleToSpeaker);

        Logger.recordOutput("FiringSolutions/CalculatedShot", solution.toString());
        Logger.recordOutput("FiringSolutions/BotPoseInput/Mag", botMagToSpeaker);
        Logger.recordOutput("FiringSolutions/BotPoseInput/Angle", botAngleToSpeaker);
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
      case AMP:
        outtake.outtake(Constants.OuttakeConstants.ampBottomShooterRPS, Constants.OuttakeConstants.ampTopShooterRPS); 
        return;
      case STOP:
      default:
        outtake.stopOuttake();
        outtake.stopPivot();
        return;
    }

    if (RobotCoordinator.getInstance().canSpinFlywheel()) {
      outtake.outtake(solution.getFlywheelSpeed(), solution.getFlywheelSpeed());
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
