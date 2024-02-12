package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intakeDeployer.IntakeDeployer;

public class IntakeDeploy extends Command {
  private final IntakeDeployer intakeDeployer;

  public IntakeDeploy() {
    intakeDeployer = IntakeDeployer.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeDeployer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().canDeploy()) {
      intakeDeployer.deploy();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotCoordinator.getInstance().isIntakeDeployed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
}
