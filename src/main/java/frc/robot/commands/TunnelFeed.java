package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tunnel.TunnelInterface;

public class TunnelFeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Used to interrupt all other drive commands and stop the drive

  private final TunnelInterface tunnel;

  public TunnelFeed(TunnelInterface tunnelSubsystem) {
    tunnel = tunnelSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tunnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tunnel.feed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
