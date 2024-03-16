package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooting.FiringSolution;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;

public class SetOuttake extends InstantCommand {
  private Outtake outtake;
  private double flywheelSpeed;
  private double pivotRotations;

  public SetOuttake(FiringSolution solution) {
    outtake = Outtake.getInstance();
    this.flywheelSpeed = solution.getFlywheelSpeed();
    this.pivotRotations = solution.getShotRotations();

    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    if (RobotCoordinator.getInstance().canPivot()) {
      outtake.pivot(pivotRotations);
    }
    if (RobotCoordinator.getInstance().canSpinFlywheel()) {
      outtake.outtake(flywheelSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
