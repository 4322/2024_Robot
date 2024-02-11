package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinatorInterface;
import frc.robot.subsystems.intake.IntakeInterface;
import frc.robot.subsystems.intakeDeployer.IntakeDeployerInterface;

public class PivotAndIntake extends Command {
  private final IntakeInterface intake;
  private final IntakeDeployerInterface intakeDeployer;
  private final RobotCoordinatorInterface coordinator;

  private boolean deployed;

  public PivotAndIntake(IntakeInterface intakeSubsystem, IntakeDeployerInterface intakeDeployerSubsystem, RobotCoordinatorInterface coordinator) {
    intake = intakeSubsystem;
    intakeDeployer = intakeDeployerSubsystem;
    this.coordinator = coordinator;

    deployed = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, intakeDeployer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (coordinator.canDeploy() && !deployed) { // only deploy once to not break PID (just in case)
      deployed = true;
      intakeDeployer.deploy();
    }
    if (coordinator.canIntake()) {
      intake.intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
