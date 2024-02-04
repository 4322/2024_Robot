package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeDeployer.IntakeDeployerInterface;

public class IntakeRetract extends Command {
  private final IntakeDeployerInterface intakeDeployer;

  public IntakeRetract(IntakeDeployerInterface deployerSubsystem) {
    intakeDeployer = deployerSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeDeployer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeDeployer.retract();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
