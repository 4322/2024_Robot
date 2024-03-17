package frc.robot.commands.DriveManual;

import com.github.oxo42.stateless4j.*;

public class DriveManualStateMachine {

  private StateMachine<DriveManualState, DriveManualTrigger> stateMachine;
  private StateMachineConfig<DriveManualState, DriveManualTrigger> config =
      new StateMachineConfig<>();

  public enum DriveManualState {
    DEFAULT,
    SPEAKER_CENTRIC,
    ROBOT_CENTRIC
  }

  public enum DriveManualTrigger {
    ENABLE_SPEAKER_CENTRIC,
    ENABLE_ROBOT_CENTRIC,
    RESET_TO_DEFAULT,
  }

  public DriveManualStateMachine(DriveManualState initialState) {
    config
        .configure(DriveManualState.DEFAULT)
        .permit(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC, DriveManualState.SPEAKER_CENTRIC)
        .permit(DriveManualTrigger.ENABLE_ROBOT_CENTRIC, DriveManualState.ROBOT_CENTRIC)
        .permitReentry(DriveManualTrigger.RESET_TO_DEFAULT);

    config
        .configure(DriveManualState.SPEAKER_CENTRIC)
        .permit(DriveManualTrigger.RESET_TO_DEFAULT, DriveManualState.DEFAULT)
        .permitReentry(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC)
        .permitReentry(
            DriveManualTrigger
                .ENABLE_ROBOT_CENTRIC); // don't want to accidentally switch to robot centric from
    // speaker centric

    config
        .configure(DriveManualState.ROBOT_CENTRIC)
        .permit(DriveManualTrigger.RESET_TO_DEFAULT, DriveManualState.DEFAULT)
        .permitReentry(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC)
        .permitReentry(
            DriveManualTrigger
                .ENABLE_ROBOT_CENTRIC); // don't want to accidentally switch to speaker centric from
    // robot centric

    stateMachine = new StateMachine<DriveManualState, DriveManualTrigger>(initialState, config);
  }

  public DriveManualState getState() {
    return stateMachine.getState();
  }

  public void fire(DriveManualTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
