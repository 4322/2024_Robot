package frc.robot.centerline;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.centerline.CLSM.CLSMState;
import frc.robot.centerline.CLSM.CLSMTrigger;
import frc.robot.subsystems.RobotCoordinator;

public class CenterLineManager {
  private final CLSM machine;
  private final EnvironmentTracker tracker;

  private boolean initialized = false;

  public enum ScoringStrategy {
    OneToFive,
    FiveToOne,
    DoNothing,
  }

  public CenterLineManager(ScoringStrategy strategy) {
    machine = new CLSM(strategy);
    tracker = new EnvironmentTracker(new NoteStatus(true, true, true, true, true));
  }

  private void updateStateMachine() {
    if (CLSM.isDrivingOnlyState(machine.getState())) {
      machine.fire(CLSMTrigger.Finished, tracker.getNoteStatus());
    } else {
      if (RobotCoordinator.getInstance().noteInRobot()) {
        machine.fire(CLSMTrigger.HaveNote, tracker.getNoteStatus());
      } else {
        machine.fire(CLSMTrigger.NoNote, tracker.getNoteStatus());
      }
    }
  }

  public Command getCommand() {
    if (!initialized) {
      machine.fire(CLSMTrigger.Initialize, new NoteStatus(true, true, true, true, true));
      initialized = true;
    } else {
      tracker.update(machine.getState());
      updateStateMachine();
    }
    return CommandBuilder.buildCommand(machine.getTravelState());
  }

  public boolean isDone() {
    return machine.getState() == CLSMState.Done;
  }
}
