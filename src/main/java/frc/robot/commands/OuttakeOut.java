package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;
// may change this to take firing solution

public class OuttakeOut extends Command {
  Outtake outtake;
  double outtakeRPM;

  public OuttakeOut(double targetOuttakeRPM) {
    outtake = Outtake.getInstance();
    outtakeRPM = targetOuttakeRPM;
    addRequirements(outtake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().isAcrossCenterLine()) {
      outtake.outtake(outtakeRPM);
    }
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
  }
}
