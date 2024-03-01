package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;

public class AutoIntakeDeploy extends Command {

  public AutoIntakeDeploy() {
    addRequirements(Intake.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().canDeploy()) {
      Intake.getInstance().deploy();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotCoordinator.getInstance().isIntakeDeployed();
  }

  @Override
  public void end(boolean interrupted) {}
}
