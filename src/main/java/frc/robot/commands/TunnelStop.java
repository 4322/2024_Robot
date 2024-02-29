package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.tunnel.Tunnel;

public class TunnelStop extends InstantCommand {
  private Tunnel tunnel;

  public TunnelStop() {
    tunnel = Tunnel.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tunnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tunnel.stopTunnel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
