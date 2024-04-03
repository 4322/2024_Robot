package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.tunnel.Tunnel;

public class Shoot extends Command {
  private final Tunnel tunnel;
  private Timer abortTimer = new Timer();

  public Shoot() {
    tunnel = Tunnel.getInstance();
    addRequirements(tunnel);
  }

  @Override
  public void initialize() {
    abortTimer.stop();
    abortTimer.reset();
  }

  @Override
  public void execute() {
    if (DriverStation.isAutonomousEnabled()) {
      if (RobotCoordinator.getInstance().canShoot()) {
        tunnel.feed();
        abortTimer.start();
      }
    } else {
      tunnel.feed();
      abortTimer.start();
    }

    if (abortTimer.hasElapsed(Constants.OuttakeConstants.shootFeedAbortSec)) {
      tunnel.stopTunnel();  // don't fry the motoe
    }

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    tunnel.stopTunnel();
  }
}
