package frc.robot.commands.OuttakeManual;

import com.github.oxo42.stateless4j.*;

public class OuttakeManualStateMachine {

  private StateMachine<OuttakeManualState, OuttakeManualTrigger> stateMachine;
  private StateMachineConfig<OuttakeManualState, OuttakeManualTrigger> config =
      new StateMachineConfig<>();

  public enum OuttakeManualState {
    SMART_SHOOTING,
    SUBWOOFER,
    EJECT,
    COLLECTING_NOTE,
    STOP
  }

  public enum OuttakeManualTrigger {
    ENABLE_SMART_SHOOTING,
    ENABLE_SUBWOOFER,
    ENABLE_EJECT,
    ENABLE_COLLECTING_NOTE,
    ENABLE_STOP
  }

  public OuttakeManualStateMachine(OuttakeManualState initialState) {
    config
        .configure(OuttakeManualState.SMART_SHOOTING)
        .permitReentry(OuttakeManualTrigger.ENABLE_SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP);

    config
        .configure(OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permitReentry(OuttakeManualTrigger.ENABLE_SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP);

    config
        .configure(OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permitReentry(OuttakeManualTrigger.ENABLE_EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP);

    config
        .configure(OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permitReentry(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP);

    config
        .configure(OuttakeManualState.STOP)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permitReentry(OuttakeManualTrigger.ENABLE_STOP);

    stateMachine = new StateMachine<OuttakeManualState, OuttakeManualTrigger>(initialState, config);
  }

  public OuttakeManualState getState() {
    return stateMachine.getState();
  }

  public void fire(OuttakeManualTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
