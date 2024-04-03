package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.tunnel.Tunnel;
import org.littletonrobotics.junction.Logger;

public class TunnelFeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Used to interrupt all other drive commands and stop the drive

  private final Tunnel tunnel;

  public enum State {
    waitForIntake,
    waitForNote,
    inTunnel,
    stoppingAtOuttake,
    rewinding,
    checkOuttakeSensor,
    pushUp,
    readyToFire,
    abort,
    waitForEject
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

    switch (state) {
      case abort:
        tunnel.stopTunnel();
        abortTimer.stop();
        abortTimer.reset();
        if (RobotCoordinator.getInstance().noteEnteringIntake()) {
          // don't keep immediately restarting the tunnel
          state = State.waitForEject;
        } else {
          state = State.waitForIntake;
        }
        break;
      case waitForEject:
        if (!RobotCoordinator.getInstance().noteEnteringIntake()) {
          state = State.waitForIntake;
        }
        break;
      case waitForIntake:
        // start tunnel at same time as intake so it gets up to speed
        if (RobotCoordinator.getInstance().intakeIsFeeding()) {
          tunnel.feed();
          state = State.waitForNote;
        }
        break;
      case waitForNote:
        if (RobotCoordinator.getInstance().noteEnteringIntake()) {
          abortTimer.start();
          state = State.inTunnel;
        } else if (!RobotCoordinator.getInstance().intakeIsFeeding()) {
          state = State.abort;  // intake stopped without seeing a note
        }
        break;
      case inTunnel:
        if (RobotCoordinator.getInstance().noteInFiringPosition()) {
          tunnel.stopTunnel();
          adjustmentTimer.restart();
          CommandScheduler.getInstance().schedule(new XboxControllerRumble());
          state = State.stoppingAtOuttake;
        } else if (abortTimer.hasElapsed(Constants.TunnelConstants.feedAbortSec)) {
          state = State.abort;  // don't fry the motor
        }
        break;
      case stoppingAtOuttake:
        if (adjustmentTimer.hasElapsed(Constants.TunnelConstants.pauseSec)) {
          tunnel.reverseFeed(); // pull back from the outtake wheels
          adjustmentTimer.restart();
          state = State.rewinding;
        }
        break;
      case rewinding:
        if (!RobotCoordinator.getInstance().noteInFiringPosition()) {
          tunnel.stopTunnel();
          adjustmentTimer.restart();
          state = State.checkOuttakeSensor;
        } else if (adjustmentTimer.hasElapsed(Constants.TunnelConstants.rewindSec)) {
          tunnel.stopTunnel();
          state = State.abort;
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
        break;
      case pushUp:
        if (RobotCoordinator.getInstance().noteInFiringPosition()) {
          tunnel.stopTunnel();
          state = State.readyToFire;
        }
        break;
      case readyToFire:
        break;
    }
    if (abortTimer.hasElapsed(Constants.TunnelConstants.totalAbortSec)) {
      state = State.abort;
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
