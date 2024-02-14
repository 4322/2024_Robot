package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;

public class IntakeIn extends Command {
  private final Intake intake;
  private boolean hasDetectedNoteInitial;

  public IntakeIn() {
    intake = Intake.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasDetectedNoteInitial = false;
  }

  @Override
  public void execute() {
    // when the note first breaks the beam
    if (RobotCoordinator.getInstance().noteAtFirstSensor()) {
      hasDetectedNoteInitial = true;
    }

    if (RobotCoordinator.getInstance().canIntake()) {
      intake.intake();
    }
  }

  @Override
  public boolean isFinished() {
    // if the note initally broke the beam and now doesn't break it, then note is secured in intake
    return hasDetectedNoteInitial && !RobotCoordinator.getInstance().noteAtFirstSensor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }
}
