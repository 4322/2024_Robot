package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
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
    // Accounts for note being midway between intake and tunnel sensor
    // Tunnel still runs for this case
    if (Intake.getInstance().getState() == IntakeStates.noteObtained) {
      noteDetected = true;
    }

    if (noteDetected) {
      tunnel.feed();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotCoordinator.getInstance().noteInFiringPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tunnel.stopTunnel();
  }
}
