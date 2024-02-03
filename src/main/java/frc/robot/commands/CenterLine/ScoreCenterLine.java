package frc.robot.commands.CenterLine;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CenterLine.stateMachine.CLSM;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.notetracker.NoteTrackerInterface;

public class ScoreCenterLine extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drive drive;
  private final CLSM machine;
  private final NoteTrackerInterface noteTracker;

  public enum ScoringStrategy {
    TopThree,
    DoNothing,
  }

  public ScoreCenterLine(Drive drivesubsystem, NoteTrackerInterface noteTracker, ScoringStrategy strategy) {
    drive = drivesubsystem;
    this.noteTracker = noteTracker;
    machine = new CLSM(strategy);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.stop();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
