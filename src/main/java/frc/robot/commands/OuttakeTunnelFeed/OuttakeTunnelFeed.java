package frc.robot.commands.OuttakeTunnelFeed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.OuttakeTunnelFeed.OuttakeTunnelFeedStateMachine.OuttakeTunnelFeedState;
import frc.robot.commands.OuttakeTunnelFeed.OuttakeTunnelFeedStateMachine.OuttakeTunnelFeedTrigger;
import frc.robot.subsystems.noteTracker.NoteTracker;
import frc.robot.subsystems.tunnel.Tunnel;

public class OuttakeTunnelFeed extends Command {
  private final Tunnel tunnel;
  private final OuttakeTunnelFeedStateMachine stateMachine;
  Timer tunnelDelayTimer = new Timer();

  public OuttakeTunnelFeed() {
    tunnel = Tunnel.getInstance();
    stateMachine = new OuttakeTunnelFeedStateMachine(OuttakeTunnelFeedState.NO_NOTE);

    addRequirements(Tunnel.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch (stateMachine.getState()) {
      case NO_NOTE:
        if (NoteTracker.getInstance().tunnelBeamBroken()) {
          stateMachine.fire(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_BROKEN);
        }
        break;
      case NOTE_PASSING_TUNNEL:
        tunnel.reverseFeed();
        if (!NoteTracker.getInstance().tunnelBeamBroken()) {
          stateMachine.fire(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN);
        }
        break;
      case NOTE_PAST_TUNNEL:
        tunnel.stopTunnel();
        tunnelDelayTimer.start();
        if (tunnelDelayTimer.hasElapsed(0.5)) {
          stateMachine.fire(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN);
        }
        break;
      case NOTE_IDLE_IN_TUNNEL:
        // delay of 0.5 needed before entering this state in order to stop tunnel from reversing
        // direction immediately and causing wear in pulleys
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return stateMachine.getState() == OuttakeTunnelFeedState.NOTE_IDLE_IN_TUNNEL;
  }

  @Override
  public void end(boolean interrupted) {
    // needed to reset state machine
    stateMachine.fire(OuttakeTunnelFeedTrigger.ENABLE_STATE_RESET);
    tunnelDelayTimer.stop();
    tunnelDelayTimer.reset();
  }
}
