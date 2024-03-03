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

public class OuttakeManual extends Command {
  private final Outtake outtake;

  private final OuttakeManualStateMachine stateMachine;

  public OuttakeManual() {
    outtake = Outtake.getInstance();
    stateMachine = new OuttakeManualStateMachine(OuttakeManualState.STOP);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    final FiringSolution solution;
    boolean limitForwardMotion = true;
    switch (stateMachine.getState()) {
      case SMART_SHOOTING:
        solution =
            FiringSolutionManager.getInstance()
                .calcSolution(
                    FiringSolutionHelper.getMag(
                        RobotCoordinator.getInstance().getRobotXPos(),
                        RobotCoordinator.getInstance().getRobotYPos()),
                    FiringSolutionHelper.getAngle(
                            RobotCoordinator.getInstance().getRobotXPos(),
                            RobotCoordinator.getInstance().getRobotYPos())
                        .getDegrees());
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
      case CLIMBING:
        solution = FiringSolutions.Climbing;
        limitForwardMotion = false;
        RobotCoordinator.getInstance().setInClimbingMode(true);
      case STOP:
      default:
        outtake.stopOuttake();
        outtake.stopPivot();
        RobotCoordinator.getInstance().setInClimbingMode(false);
        return;
    }

    if (RobotCoordinator.getInstance().canSpinFlywheel()) {
      outtake.outtake(solution.getFlywheelSpeed());
    } else {
      outtake.stopOuttake();
    }

    if (RobotCoordinator.getInstance().canPivot()) {
      outtake.pivot(solution.getShotRotations(), limitForwardMotion);
    } else {
      outtake.stopPivot();
    }
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
