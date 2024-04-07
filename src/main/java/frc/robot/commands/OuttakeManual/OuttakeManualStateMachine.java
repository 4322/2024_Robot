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
    STOP,
    FEED,
    AMP,
    STARTING_CONFIG,
    PASS;
  }

  public enum OuttakeManualTrigger {
    ENABLE_SMART_SHOOTING,
    ENABLE_SUBWOOFER,
    ENABLE_EJECT,
    ENABLE_COLLECTING_NOTE,
    ENABLE_STOP,
    ENABLE_FEED, 
    ENABLE_AMP,
    ENABLE_STARTING_CONFIG,
    ENABLE_PASS;
  }

  public OuttakeManualStateMachine(OuttakeManualState initialState) {
    config
        .configure(OuttakeManualState.SMART_SHOOTING)
        .permitReentry(OuttakeManualTrigger.ENABLE_SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP)
        .permit(OuttakeManualTrigger.ENABLE_FEED, OuttakeManualState.FEED)
        .permit(OuttakeManualTrigger.ENABLE_AMP, OuttakeManualState.AMP)
        .permit(OuttakeManualTrigger.ENABLE_STARTING_CONFIG, OuttakeManualState.STARTING_CONFIG)
        .permit(OuttakeManualTrigger.ENABLE_PASS, OuttakeManualState.PASS);

    config
        .configure(OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permitReentry(OuttakeManualTrigger.ENABLE_SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP)
        .permit(OuttakeManualTrigger.ENABLE_FEED, OuttakeManualState.FEED)
        .permit(OuttakeManualTrigger.ENABLE_AMP, OuttakeManualState.AMP)
        .permit(OuttakeManualTrigger.ENABLE_STARTING_CONFIG, OuttakeManualState.STARTING_CONFIG)
        .permit(OuttakeManualTrigger.ENABLE_PASS, OuttakeManualState.PASS);

    config
        .configure(OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permitReentry(OuttakeManualTrigger.ENABLE_EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP)
        .permit(OuttakeManualTrigger.ENABLE_FEED, OuttakeManualState.FEED)
        .permit(OuttakeManualTrigger.ENABLE_AMP, OuttakeManualState.AMP)
        .permit(OuttakeManualTrigger.ENABLE_STARTING_CONFIG, OuttakeManualState.STARTING_CONFIG)
        .permit(OuttakeManualTrigger.ENABLE_PASS, OuttakeManualState.PASS);

    // lockout of presets until the note is safely in the outtake
    //
    // the state will change in the OuttakeManual state machine when
    // either a note triggers the tunnel sensor or the command ends
    // from releasing the physical trigger
    config
        .configure(OuttakeManualState.COLLECTING_NOTE)
        .permitReentry(OuttakeManualTrigger.ENABLE_SMART_SHOOTING)
        .permitReentry(OuttakeManualTrigger.ENABLE_SUBWOOFER)
        .permitReentry(OuttakeManualTrigger.ENABLE_EJECT)
        .permitReentry(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP)
        .permit(OuttakeManualTrigger.ENABLE_FEED, OuttakeManualState.FEED)
        .permitReentry(OuttakeManualTrigger.ENABLE_AMP)
        .permit(OuttakeManualTrigger.ENABLE_STARTING_CONFIG, OuttakeManualState.STARTING_CONFIG)
        .permit(OuttakeManualTrigger.ENABLE_PASS, OuttakeManualState.PASS);

    config
        .configure(OuttakeManualState.STOP)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permitReentry(OuttakeManualTrigger.ENABLE_STOP)
        .permit(OuttakeManualTrigger.ENABLE_FEED, OuttakeManualState.FEED)
        .permit(OuttakeManualTrigger.ENABLE_AMP, OuttakeManualState.AMP)
        .permit(OuttakeManualTrigger.ENABLE_STARTING_CONFIG, OuttakeManualState.STARTING_CONFIG)
        .permit(OuttakeManualTrigger.ENABLE_PASS, OuttakeManualState.PASS);

    config
        .configure(OuttakeManualState.FEED)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP)
        .permitReentry(OuttakeManualTrigger.ENABLE_FEED)
        .permit(OuttakeManualTrigger.ENABLE_AMP, OuttakeManualState.AMP)
        .permit(OuttakeManualTrigger.ENABLE_STARTING_CONFIG, OuttakeManualState.STARTING_CONFIG)
        .permit(OuttakeManualTrigger.ENABLE_PASS, OuttakeManualState.PASS);

    config.configure(OuttakeManualState.AMP)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP)
        .permit(OuttakeManualTrigger.ENABLE_FEED, OuttakeManualState.FEED)
        .permitReentry(OuttakeManualTrigger.ENABLE_AMP)
        .permit(OuttakeManualTrigger.ENABLE_STARTING_CONFIG, OuttakeManualState.STARTING_CONFIG)
        .permit(OuttakeManualTrigger.ENABLE_PASS, OuttakeManualState.PASS);

    config.configure(OuttakeManualState.STARTING_CONFIG)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP)
        .permit(OuttakeManualTrigger.ENABLE_FEED, OuttakeManualState.FEED)
        .permit(OuttakeManualTrigger.ENABLE_AMP, OuttakeManualState.AMP)
        .permitReentry(OuttakeManualTrigger.ENABLE_STARTING_CONFIG)
        .permit(OuttakeManualTrigger.ENABLE_PASS, OuttakeManualState.PASS);
    
    config.configure(OuttakeManualState.PASS)
        .permit(OuttakeManualTrigger.ENABLE_SMART_SHOOTING, OuttakeManualState.SMART_SHOOTING)
        .permit(OuttakeManualTrigger.ENABLE_SUBWOOFER, OuttakeManualState.SUBWOOFER)
        .permit(OuttakeManualTrigger.ENABLE_EJECT, OuttakeManualState.EJECT)
        .permit(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE, OuttakeManualState.COLLECTING_NOTE)
        .permit(OuttakeManualTrigger.ENABLE_STOP, OuttakeManualState.STOP)
        .permit(OuttakeManualTrigger.ENABLE_FEED, OuttakeManualState.FEED)
        .permit(OuttakeManualTrigger.ENABLE_AMP, OuttakeManualState.AMP)
        .permit(OuttakeManualTrigger.ENABLE_STARTING_CONFIG, OuttakeManualState.STARTING_CONFIG)
        .permitReentry(OuttakeManualTrigger.ENABLE_PASS);

    stateMachine = new StateMachine<OuttakeManualState, OuttakeManualTrigger>(initialState, config);
  }

  public OuttakeManualState getState() {
    return stateMachine.getState();
  }

  public void fire(OuttakeManualTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
