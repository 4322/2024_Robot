package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intakeDeployer.IntakeDeployerInterface;

public class IntakeDeploy extends InstantCommand {
  private final IntakeDeployerInterface intakeDeployer;

  public IntakeDeploy(IntakeDeployerInterface deployerSubsystem) {
    intakeDeployer = deployerSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeDeployer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeDeployer.deploy();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}