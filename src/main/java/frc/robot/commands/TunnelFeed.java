package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

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
    waitForIntake,
    inTunnel,
    stoppingAtOuttake,
    rewinding,
    checkOuttakeSensor,
    pushUp,
    readyToFire,
    abort
  }

  private State state;
  private Timer adjustmentTimer = new Timer();
  private Timer abortTimer = new Timer();

  public TunnelFeed() {
    tunnel = Tunnel.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tunnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.waitForIntake;
    abortTimer.stop();
    abortTimer.reset();
  }

  @Override
  public void execute() {

    if (abortTimer.hasElapsed(Constants.TunnelConstants.abortSec)) {
      tunnel.stopTunnel();
      state = State.abort;
      abortTimer.stop();
      abortTimer.reset();
    }

    switch (state) {
      case abort:
        state = State.waitForIntake;
        break;
      case waitForIntake:
        // Accounts for note being midway between intake and tunnel sensor
        // Tunnel still runs for this case
        if (RobotCoordinator.getInstance().noteEnteringIntake()) {
          tunnel.feed();
          abortTimer.start();
          state = State.inTunnel;
        }
        break;
      case inTunnel:
        if (RobotCoordinator.getInstance().noteInFiringPosition()) {
          tunnel.stopTunnel();
          adjustmentTimer.restart();
          state = State.stoppingAtOuttake;
        }
        break;
      case stoppingAtOuttake:
        if (adjustmentTimer.hasElapsed(Constants.TunnelConstants.pauseSec)) {
          tunnel.rewind(); // pull back from the outtake wheels
          adjustmentTimer.restart();
          state = State.rewinding;
        }
      case rewinding:
        if (adjustmentTimer.hasElapsed(Constants.TunnelConstants.rewindSec)) {
          tunnel.stopTunnel();
          adjustmentTimer.restart();
          state = State.checkOuttakeSensor;
        }
        break;
      case checkOuttakeSensor:
        if (adjustmentTimer.hasElapsed(Constants.TunnelConstants.pauseSec)) {
          if (RobotCoordinator.getInstance().noteInFiringPosition()) {
            state = State.readyToFire;
          } else {
            tunnel.pushUp();
            adjustmentTimer.restart();
            state = State.pushUp;
          }
        }
      case pushUp:
      if (RobotCoordinator.getInstance().noteInFiringPosition()) {
        tunnel.stopTunnel();
        state = State.readyToFire;
      }
      case readyToFire:
      break;
    }
    Logger.recordOutput("TunnelFeed/State", state.toString());
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
    Logger.recordOutput("TunnelFeed/State", "ended");
  }
}
