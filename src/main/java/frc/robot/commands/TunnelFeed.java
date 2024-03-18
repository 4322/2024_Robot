package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.tunnel.Tunnel;

public class TunnelFeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Used to interrupt all other drive commands and stop the drive

  private final Tunnel tunnel;

  public enum State {
    intakeControl,
    inTunnel,
    stoppingAtOuttake,
    rewinding,
    readyToFire
  }

  private State state;
  private Timer timer = new Timer();

  public TunnelFeed() {
    tunnel = Tunnel.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tunnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.intakeControl;
  }

  @Override
  public void execute() {

    switch (state) {
      case intakeControl:
        // Accounts for note being midway between intake and tunnel sensor
        // Tunnel still runs for this case
        if (RobotCoordinator.getInstance().noteEnteringIntake()) {
          tunnel.feed();
          state = State.inTunnel;
        }
        break;
      case inTunnel:
        if (RobotCoordinator.getInstance().noteInFiringPosition()) {
          tunnel.stopTunnel();
          timer.restart();
          state = State.stoppingAtOuttake;
        }
        break;
      case stoppingAtOuttake:
        if (timer.hasElapsed(Constants.TunnelConstants.pauseSec)) {
          tunnel.rewind(); // pull back from the outtake wheels
          timer.restart();
          state = State.rewinding;
        }
      case rewinding:
        if (timer.hasElapsed(Constants.TunnelConstants.rewindSec)) {
          tunnel.stopTunnel();
          state = State.readyToFire;
        }
        break;
      case readyToFire:
        break;
    }
  }

  @Override
  public boolean isFinished() {
    // never end a default command
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tunnel.stopTunnel();
  }
}
