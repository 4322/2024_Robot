package frc.robot.commands.OuttakeTunnelFeed;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

public class OuttakeTunnelFeedStateMachine {
  private StateMachine<OuttakeTunnelFeedState, OuttakeTunnelFeedTrigger> stateMachine;
  private StateMachineConfig<OuttakeTunnelFeedState, OuttakeTunnelFeedTrigger> config =
      new StateMachineConfig<>();

  public enum OuttakeTunnelFeedState {
    NO_NOTE,
    NOTE_ENTERING_TUNNEL,
    NOTE_IN_TUNNEL,
    NOTE_PUSHING_UP
  }

  public enum OuttakeTunnelFeedTrigger {
    TUNNEL_BEAM_NOT_BROKEN,
    TUNNEL_BEAM_BROKEN,
    STOP_DELAY_DETECTED,
    ENABLE_STATE_RESET
  }

  public OuttakeTunnelFeedStateMachine(OuttakeTunnelFeedState initialState) {
    config
        .configure(OuttakeTunnelFeedState.NO_NOTE)
        .permit(
            OuttakeTunnelFeedTrigger.TUNNEL_BEAM_BROKEN, OuttakeTunnelFeedState.NOTE_ENTERING_TUNNEL)
        .permitReentry(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN)
        .permitReentry(OuttakeTunnelFeedTrigger.ENABLE_STATE_RESET)
        .permitReentry(OuttakeTunnelFeedTrigger.STOP_DELAY_DETECTED);

    config
        .configure(OuttakeTunnelFeedState.NOTE_ENTERING_TUNNEL)
        .permit(
            OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN,
            OuttakeTunnelFeedState.NOTE_IN_TUNNEL)
        .permit(OuttakeTunnelFeedTrigger.ENABLE_STATE_RESET, OuttakeTunnelFeedState.NO_NOTE)
        .permitReentry(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_BROKEN)
        .permitReentry(OuttakeTunnelFeedTrigger.STOP_DELAY_DETECTED);

    config
        .configure(OuttakeTunnelFeedState.NOTE_IN_TUNNEL)
        .permit(
            OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN,
            OuttakeTunnelFeedState.NOTE_PUSHING_UP)
        .permit(OuttakeTunnelFeedTrigger.ENABLE_STATE_RESET, OuttakeTunnelFeedState.NO_NOTE)
        .permit(OuttakeTunnelFeedTrigger.STOP_DELAY_DETECTED, OuttakeTunnelFeedState.NOTE_ENTERING_TUNNEL)
        .permitReentry(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_BROKEN);

    config
        .configure(OuttakeTunnelFeedState.NOTE_PUSHING_UP)
        .permit(OuttakeTunnelFeedTrigger.ENABLE_STATE_RESET, OuttakeTunnelFeedState.NO_NOTE)
        .permitReentry(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_BROKEN)
        .permitReentry(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN)
        .permitReentry(OuttakeTunnelFeedTrigger.STOP_DELAY_DETECTED);

    stateMachine =
        new StateMachine<OuttakeTunnelFeedState, OuttakeTunnelFeedTrigger>(initialState, config);
  }

  public OuttakeTunnelFeedState getState() {
    return stateMachine.getState();
  }

  public void fire(OuttakeTunnelFeedTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
