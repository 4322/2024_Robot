package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.FiringSolutionHelper;

public class OuttakeSubwoofer extends Command {
  Outtake outtake;

  public OuttakeSubwoofer() {
    outtake = Outtake.getInstance();

    addRequirements(outtake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().onOurSideOfField() && outtake.safeToPivot()) {
      outtake.outtake(OuttakeConstants.subwooferOuttakeRPS);
      outtake.pivot(OuttakeConstants.subwooferPivotPositionRotations);
    }
  }

  @Override
  public boolean isFinished() {
    return !RobotCoordinator.getInstance().onOurSideOfField();
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
    outtake.stopPivot();
  }
}
