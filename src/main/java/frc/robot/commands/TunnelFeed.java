package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.tunnel.Tunnel;

public class TunnelFeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Used to interrupt all other drive commands and stop the drive

  private final Tunnel tunnel;
  private boolean noteDetected;

  public TunnelFeed() {
    tunnel = Tunnel.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tunnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteDetected = false;
  }

  @Override
  public void execute() {
    // If note midway between intake and tunnel sensor, tunnel still runs
    if (RobotCoordinator.getInstance().noteAtIntakeSensor()) {
      noteDetected = true;
    }

    if (noteDetected) {
      tunnel.feed();
    }
    
  }

  @Override
  public boolean isFinished() {
    return RobotCoordinator.getInstance().noteAtTunnelSensor() && !RobotCoordinator.getInstance().noteAtIntakeSensor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tunnel.stopTunnel();
  }
}
