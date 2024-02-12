package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;

public class IntakeIn extends Command {
  private final Intake intake;
  private boolean hasNote;

  public IntakeIn() {
    intake = Intake.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().noteAtFirstSensor()) {
      hasNote = true;
    }

    if (RobotCoordinator.getInstance().canIntake()) {
      intake.intake();
    }
  }

  @Override
  public boolean isFinished() {
    return hasNote && !RobotCoordinator.getInstance().noteAtFirstSensor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    hasNote = false;
  }
}
