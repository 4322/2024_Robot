package frc.robot.commands.OuttakeTunnelFeed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ControllerRumbleTimes;
import frc.robot.commands.DriverXboxControllerRumble;
import frc.robot.commands.OuttakeManual.OuttakeManual;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualState;
import frc.robot.commands.OuttakeTunnelFeed.OuttakeTunnelFeedStateMachine.OuttakeTunnelFeedState;
import frc.robot.commands.OuttakeTunnelFeed.OuttakeTunnelFeedStateMachine.OuttakeTunnelFeedTrigger;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.noteTracker.NoteTracker;
import frc.robot.subsystems.tunnel.Tunnel;

public class OuttakeTunnelFeed extends Command {
  private final Tunnel tunnel;
  private final OuttakeTunnelFeedStateMachine stateMachine;
  Timer tunnelDelayTimer = new Timer();
  Timer restartFeedTimer = new Timer();

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
          CommandScheduler.getInstance().schedule(new DriverXboxControllerRumble(ControllerRumbleTimes.longRumbleTime));
          stateMachine.fire(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_BROKEN);
        }
        break;
      case NOTE_ENTERING_TUNNEL:
        tunnel.reverseFeed();
        if (!NoteTracker.getInstance().tunnelBeamBroken()) {
          stateMachine.fire(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN);
        }
        break;
      case NOTE_IN_TUNNEL:
        tunnel.stopTunnel();
        if (!tunnel.isStopped(0.1)) {
          restartFeedTimer.start();
        } else {
          restartFeedTimer.stop();
        }
        tunnelDelayTimer.start();
        if (tunnelDelayTimer.hasElapsed(0.5)) {
          stateMachine.fire(OuttakeTunnelFeedTrigger.TUNNEL_BEAM_NOT_BROKEN);
        }
        if (restartFeedTimer.hasElapsed(0.160)) {
          restartFeedTimer.stop();
          restartFeedTimer.reset();
          stateMachine.fire(OuttakeTunnelFeedTrigger.STOP_DELAY_DETECTED);
        }
        break;
      case NOTE_PUSHING_UP:
        // delay of 0.5 needed before entering this state in order to stop tunnel from reversing
        // direction immediately and causing wear in pulleys
        tunnel.pushUp();
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return (stateMachine.getState() == OuttakeTunnelFeedState.NOTE_PUSHING_UP
            && RobotCoordinator.getInstance().noteInFiringPosition())
        || (OuttakeManual.getState() != OuttakeManualState.FEED);
  }

  @Override
  public void end(boolean interrupted) {
    // needed to reset state machine
    stateMachine.fire(OuttakeTunnelFeedTrigger.ENABLE_STATE_RESET);
    tunnel.stopTunnel();

    tunnelDelayTimer.stop();
    tunnelDelayTimer.reset();
    restartFeedTimer.stop();
    restartFeedTimer.reset();
  }
}
