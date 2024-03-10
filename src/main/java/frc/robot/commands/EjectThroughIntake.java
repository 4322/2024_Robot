package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeManual.IntakeStates;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.tunnel.Tunnel;

public class EjectThroughIntake extends Command {

  public EjectThroughIntake() {
    addRequirements(Intake.getInstance(), Tunnel.getInstance());
  }

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().intakeIsDeployed()) {
      Intake.getInstance().outtake();
      Tunnel.getInstance().reverseFeed();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    IntakeManual.setIntakeState(IntakeStates.deploying);
    Tunnel.getInstance().stopTunnel();
  }
}
