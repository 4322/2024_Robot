package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;

public class SetOuttakeAdjust extends InstantCommand {
  private Outtake outtake;
  private double flywheelSpeed;
  private double pivotAngle;

  public SetOuttakeAdjust(double flywheelSpeed, double pivotAngle) {
    outtake = Outtake.getInstance();
    this.flywheelSpeed = flywheelSpeed;
    this.pivotAngle = pivotAngle;

    addRequirements(outtake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().canPivot()) {
      outtake.outtake(flywheelSpeed);
      outtake.pivot(pivotAngle);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
    outtake.stopPivot();
  }
}
