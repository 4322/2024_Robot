package frc.robot.commands.DriveAuto;

import com.github.oxo42.stateless4j.*;

public class DriveAutoStateMachine {
  private StateMachine<DriveAutoState, DriveAutoTrigger> stateMachine;
  private StateMachineConfig<DriveAutoState, DriveAutoTrigger> config = new StateMachineConfig<>();

  public enum DriveAutoState {
    DEFAULT,
    NOTE,
    NO_NOTE
  }

  public enum DriveAutoTrigger {
    HAS_NOTE,
    LACKS_NOTE,
  }

  public DriveAutoStateMachine(DriveAutoState initialState) {
    config
        .configure(DriveAutoState.DEFAULT)
        .permit(DriveAutoTrigger.HAS_NOTE, DriveAutoState.NOTE)
        .permit(DriveAutoTrigger.LACKS_NOTE, DriveAutoState.NO_NOTE);

    stateMachine = new StateMachine<DriveAutoState, DriveAutoTrigger>(initialState, config);
  }

  public DriveAutoState getState() {
    return stateMachine.getState();
  }

  public void fire(DriveAutoTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
