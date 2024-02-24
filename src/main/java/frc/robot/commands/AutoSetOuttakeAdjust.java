package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;

public class AutoSetOuttakeAdjust extends InstantCommand {
  private Outtake outtake;
  private double flywheelSpeed;
  private double pivotAngle;

  public AutoSetOuttakeAdjust(double flywheelSpeed, double pivotAngle) {
    outtake = Outtake.getInstance();
    this.flywheelSpeed = flywheelSpeed;
    this.pivotAngle = pivotAngle;

    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    if (RobotCoordinator.getInstance().canPivot()) {
      outtake.outtake(flywheelSpeed);
      outtake.pivot(pivotAngle);
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
