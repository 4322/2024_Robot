package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intakeDeployer.IntakeDeployer;

public class IntakeRetract extends InstantCommand {
  private final IntakeDeployer intakeDeployer;

  public IntakeRetract() {
    intakeDeployer = IntakeDeployer.getInstance();

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
