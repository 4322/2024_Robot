package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.OuttakeManual.OuttakeManual;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualState;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.tunnel.Tunnel;

public class Shoot extends Command {
  private final Tunnel tunnel;

  public Shoot() {
    tunnel = Tunnel.getInstance();
    addRequirements(tunnel);
  }

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().canShoot()
        || RobotCoordinator.getInstance().debugOuttakeOverride()
        || OuttakeManual.getState() == OuttakeManualState.EJECT) {
      tunnel.feed();
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
