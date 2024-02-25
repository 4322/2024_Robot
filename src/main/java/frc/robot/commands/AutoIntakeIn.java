package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.tunnel.Tunnel;

public class AutoIntakeIn extends Command {

  public AutoIntakeIn() {
    addRequirements(Intake.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().isIntakeDeployed()) {
      Intake.getInstance().intake();
      Tunnel.getInstance().feed();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotCoordinator.getInstance().intakeIsFeeding() && RobotCoordinator.getInstance().noteInFiringPosition();
  }

  @Override
  public void end(boolean interrupted) {
    Tunnel.getInstance().stopTunnel();
  }
}
