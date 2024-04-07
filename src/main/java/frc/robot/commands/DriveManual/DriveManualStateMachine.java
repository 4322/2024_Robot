package frc.robot.commands.DriveManual;

import com.github.oxo42.stateless4j.*;

public class DriveManualStateMachine {

  private StateMachine<DriveManualState, DriveManualTrigger> stateMachine;
  private StateMachineConfig<DriveManualState, DriveManualTrigger> config =
      new StateMachineConfig<>();

  public enum DriveManualState {
    DEFAULT,
    SPEAKER_CENTRIC,
    ROBOT_CENTRIC,
    AMP,
    SOURCE,
    PASS;
  }

  public enum DriveManualTrigger {
    ENABLE_SPEAKER_CENTRIC,
    ENABLE_ROBOT_CENTRIC,
    ENABLE_AMP,
    ENABLE_SOURCE,
    ENABLE_PASS,
    RESET_TO_DEFAULT
  }

  public DriveManualStateMachine(DriveManualState initialState) {
    config
        .configure(DriveManualState.DEFAULT)
        .permit(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC, DriveManualState.SPEAKER_CENTRIC)
        .permit(DriveManualTrigger.ENABLE_ROBOT_CENTRIC, DriveManualState.ROBOT_CENTRIC)
        .permit(DriveManualTrigger.ENABLE_AMP, DriveManualState.AMP)
        .permit(DriveManualTrigger.ENABLE_SOURCE, DriveManualState.SOURCE)
        .permitReentry(DriveManualTrigger.RESET_TO_DEFAULT)
        .permit(DriveManualTrigger.ENABLE_PASS, DriveManualState.PASS);

    config
        .configure(DriveManualState.SPEAKER_CENTRIC)
        .permit(DriveManualTrigger.RESET_TO_DEFAULT, DriveManualState.DEFAULT)
        .permitReentry(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC)
        .permitReentry(
            DriveManualTrigger
                .ENABLE_ROBOT_CENTRIC)
        .permitReentry(DriveManualTrigger.ENABLE_AMP)
        .permitReentry(DriveManualTrigger.ENABLE_SOURCE)
        .permitReentry(DriveManualTrigger.ENABLE_PASS);

    config
        .configure(DriveManualState.ROBOT_CENTRIC)
        .permit(DriveManualTrigger.RESET_TO_DEFAULT, DriveManualState.DEFAULT)
        .permitReentry(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC)
        .permitReentry(
            DriveManualTrigger
                .ENABLE_ROBOT_CENTRIC)
        .permitReentry(DriveManualTrigger.ENABLE_AMP)
        .permitReentry(DriveManualTrigger.ENABLE_SOURCE)
        .permitReentry(DriveManualTrigger.ENABLE_PASS);
    
    config.configure(DriveManualState.AMP)
      .permit(DriveManualTrigger.RESET_TO_DEFAULT, DriveManualState.DEFAULT)
      .permitReentry(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC)
      .permitReentry(
          DriveManualTrigger
              .ENABLE_ROBOT_CENTRIC)
      .permitReentry(DriveManualTrigger.ENABLE_AMP)
      .permitReentry(DriveManualTrigger.ENABLE_SOURCE)
      .permitReentry(DriveManualTrigger.ENABLE_PASS);

    config.configure(DriveManualState.SOURCE)
      .permit(DriveManualTrigger.RESET_TO_DEFAULT, DriveManualState.DEFAULT)
      .permitReentry(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC)
      .permitReentry(
          DriveManualTrigger
              .ENABLE_ROBOT_CENTRIC)
      .permitReentry(DriveManualTrigger.ENABLE_AMP)
      .permitReentry(DriveManualTrigger.ENABLE_SOURCE)
      .permitReentry(DriveManualTrigger.ENABLE_PASS);
    
    config.configure(DriveManualState.PASS)
      .permit(DriveManualTrigger.RESET_TO_DEFAULT, DriveManualState.DEFAULT)
      .permitReentry(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC)
      .permitReentry(
          DriveManualTrigger
              .ENABLE_ROBOT_CENTRIC)
      .permitReentry(DriveManualTrigger.ENABLE_AMP)
      .permitReentry(DriveManualTrigger.ENABLE_SOURCE)
      .permitReentry(DriveManualTrigger.ENABLE_PASS);

    stateMachine = new StateMachine<DriveManualState, DriveManualTrigger>(initialState, config);
  }

  public DriveManualState getState() {
    return stateMachine.getState();
  }

  public void fire(DriveManualTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
