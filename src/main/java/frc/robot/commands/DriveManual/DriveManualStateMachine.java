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
    JOYSTICK_IN
  }

  public DriveManualStateMachine(DriveManualState initialState) {
    config
        .configure(DriveManualState.DEFAULT)
        .permit(DriveManualTrigger.JOYSTICK_IN, DriveManualState.SPEAKER_CENTRIC);

    config
        .configure(DriveManualState.SPEAKER_CENTRIC)
        .permit(DriveManualTrigger.JOYSTICK_IN, DriveManualState.DEFAULT);

    stateMachine = new StateMachine<DriveManualState, DriveManualTrigger>(initialState, config);
  }

  public DriveManualState getState() {
    return stateMachine.getState();
  }

  public void fire(DriveManualTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
