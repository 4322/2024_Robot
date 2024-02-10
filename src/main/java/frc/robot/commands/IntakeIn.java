package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeInterface;

public class IntakeIn extends InstantCommand {
  private final IntakeInterface intake;

  public IntakeIn(IntakeInterface intakeSubsystem) {
    intake = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }
}
