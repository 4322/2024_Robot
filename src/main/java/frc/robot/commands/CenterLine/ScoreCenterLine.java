package frc.robot.commands.CenterLine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CenterLine.environmentTracker.EnvironmentTracker;
import frc.robot.commands.CenterLine.environmentTracker.NoteStatus;
import frc.robot.commands.CenterLine.stateMachine.CLSM;
import frc.robot.commands.CenterLine.stateMachine.CLSM.CLSMTrigger;
import frc.robot.subsystems.drive.Drive;

public class ScoreCenterLine extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drive drive;
  private final CLSM machine;
  private final EnvironmentTracker tracker;

  private Command travelCommand;


  public enum ScoringStrategy {
    OneToFive,
    DoNothing,
  }

  public ScoreCenterLine(Drive drivesubsystem, ScoringStrategy strategy) {
    drive = drivesubsystem;
    machine = new CLSM(strategy);
    tracker = new EnvironmentTracker(new NoteStatus(true, true, true, true, true));

    travelCommand = Commands.none();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    machine.fire(CLSMTrigger.Initialize, new NoteStatus(true, true, true, true, true));
    travelCommand.execute(); // "Finish" command
  }

  @Override
  public void execute() {
    if (travelCommand.isFinished()) {
      tracker.update(machine.getState());    
      
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
