package frc.robot.commands.CenterLine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CenterLine.environmentTracker.EnvironmentTracker;
import frc.robot.commands.CenterLine.environmentTracker.NoteStatus;
import frc.robot.commands.CenterLine.statemachine.CLSM;
import frc.robot.commands.CenterLine.statemachine.CLSM.CLSMState;
import frc.robot.commands.CenterLine.statemachine.CLSM.CLSMTrigger;
import frc.robot.subsystems.RobotCoordinatorInterface;
import frc.robot.subsystems.drive.DriveInterface;

public class ScoreCenterLine extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveInterface drive;

  private final CLSM machine;
  private final EnvironmentTracker tracker;
  private final RobotCoordinatorInterface coordinator;

  private Command travelCommand;
  private boolean isFinished = false;

  public enum ScoringStrategy {
    OneToFive,
    DoNothing,
  }

  public ScoreCenterLine(
      DriveInterface drivesubsystem,
      RobotCoordinatorInterface coordinator,
      ScoringStrategy strategy) {
    drive = drivesubsystem;
    this.coordinator = coordinator;
    machine = new CLSM(strategy);
    tracker = new EnvironmentTracker(new NoteStatus(true, true, true, true, true));

    travelCommand = Commands.none();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    machine.fire(CLSMTrigger.Initialize, new NoteStatus(true, true, true, true, true));
    travelCommand.schedule(); // "Finish" command
  }

  @Override
  public void execute() {
    if (travelCommand.isFinished()) {
      tracker.update(machine.getState());
      if (isFiringState(machine.getState())) {
        machine.fire(CLSMTrigger.FinishedShooting, tracker.getNoteStatus());
      } else {
        if (coordinator.hasNote()) {
          machine.fire(CLSMTrigger.HaveNote, tracker.getNoteStatus());
        } else {
          machine.fire(CLSMTrigger.NoNote, tracker.getNoteStatus());
        }
      }
      if (machine.getState() == CLSMState.Done) {
        isFinished = true;
      } else {
        travelCommand = CommandBuilder.buildCommand(machine.getTravelState());
        travelCommand.schedule();
      }
    }
  }

  private boolean isFiringState(CLSMState state) {
    switch (state) {
      case UpperShoot:
      case MiddleShoot:
      case BottomShoot:
        return true;
      default:
        return false;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
