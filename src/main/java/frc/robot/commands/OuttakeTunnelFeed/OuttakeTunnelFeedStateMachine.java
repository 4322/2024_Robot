package frc.robot.commands.OuttakeTunnelFeed;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

public class OuttakeTunnelFeedStateMachine {
private StateMachine<OuttakeTunnelFeedState, OuttakeTunnelFeedTrigger> stateMachine;
  private StateMachineConfig<OuttakeTunnelFeedState, OuttakeTunnelFeedTrigger> config =
      new StateMachineConfig<>();

  public enum OuttakeTunnelFeedState {
    NO_NOTE,
    NOTE_PASSING_TUNNEL,
    NOTE_PAST_TUNNEL,
    NOTE_IDLE_IN_TUNNEL
  }

  public enum OuttakeTunnelFeedTrigger {
    TUNNEL_BEAM_NOT_BROKEN,
    TUNNEL_BEAM_BROKEN,
    ENABLE_STATE_RESET
  }

  public OuttakeTunnelFeedStateMachine(OuttakeTunnelFeedState initialState) {
    config
        .configure(OuttakeTunnelFeedState.NO_NOTE)
        .permit(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_BROKEN, OuttakeTunnelFeedState.NOTE_PASSING_TUNNEL);
    
    config
        .configure(OuttakeTunnelFeedState.NOTE_PASSING_TUNNEL)
        .permit(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN, OuttakeTunnelFeedState.NOTE_PAST_TUNNEL);
    
    config
        .configure(OuttakeTunnelFeedState.NOTE_PAST_TUNNEL)
        .permit(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN, OuttakeTunnelFeedState.NOTE_IDLE_IN_TUNNEL);
    
    config
        .configure(OuttakeTunnelFeedState.NOTE_IDLE_IN_TUNNEL)
        .permit(OuttakeTunnelFeedTrigger.ENABLE_STATE_RESET, OuttakeTunnelFeedState.NO_NOTE);
        

    stateMachine = new StateMachine<OuttakeTunnelFeedState, OuttakeTunnelFeedTrigger>(initialState, config);
  }

  public OuttakeTunnelFeedState getState() {
    return stateMachine.getState();
  }

  public void fire(OuttakeTunnelFeedTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
