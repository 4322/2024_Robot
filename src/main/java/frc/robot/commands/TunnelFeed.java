package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.tunnel.Tunnel;

public class TunnelFeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Used to interrupt all other drive commands and stop the drive

  private final Tunnel tunnel;
  private final RobotCoordinator coordinator;

  public TunnelFeed() {
    tunnel = Tunnel.getInstance();
    coordinator = RobotCoordinator.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tunnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (coordinator.hasNote() && !coordinator.noteInFiringPosition()) {
      // PID here
      tunnel.feedToShootingPos(); // give PID result
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
