package frc.robot.commands.DriveManual;

import com.github.oxo42.stateless4j.*;

public class DriveManualStateMachine {

  private StateMachine<DriveManualState, DriveManualTrigger> stateMachine;
  private StateMachineConfig<DriveManualState, DriveManualTrigger> config =
      new StateMachineConfig<>();

  public enum DriveManualState {
    DEFAULT,
    SPEAKER_CENTRIC
  }

  public enum DriveManualTrigger {
    SWITCH_MODES,
    RESET_TO_DEFAULT,
  }

  public DriveManualStateMachine(DriveManualState initialState) {
    config
        .configure(DriveManualState.DEFAULT)
        .permit(DriveManualTrigger.SWITCH_MODES, DriveManualState.SPEAKER_CENTRIC)
        .permitReentry(DriveManualTrigger.RESET_TO_DEFAULT);

    config
        .configure(DriveManualState.SPEAKER_CENTRIC)
        .permit(DriveManualTrigger.SWITCH_MODES, DriveManualState.DEFAULT)
        .permit(DriveManualTrigger.RESET_TO_DEFAULT, DriveManualState.DEFAULT);

    stateMachine = new StateMachine<DriveManualState, DriveManualTrigger>(initialState, config);
  }

  public DriveManualState getState() {
    return stateMachine.getState();
  }

  public void fire(DriveManualTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
