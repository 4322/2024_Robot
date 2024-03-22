package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeManual.IntakeStates;
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
  public void end(boolean interrupted) {
    // set intake manual state to deployed to make sure state machine is accurate
    // and so that Intake Manual doesn't interfere with this command by retracting
    IntakeManual.setIntakeState(IntakeStates.deployed);
  }
}
