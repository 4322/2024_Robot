package frc.robot.commands.OuttakeManual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FiringSolutions;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualState;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualTrigger;
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
      outtake.pivot(solution.getShotRotations());
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
