package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.shooting.FiringSolution;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;

public class AutoSetOuttakeAdjust extends InstantCommand {
  private Outtake outtake;
  private double flywheelSpeed;
  private double pivotAngle;

  public AutoSetOuttakeAdjust(FiringSolution solution) {
    outtake = Outtake.getInstance();
    this.flywheelSpeed = solution.getFlywheelSpeed();
    this.pivotAngle = solution.getShotRotations();

    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    if (RobotCoordinator.getInstance().canPivot()) {
      outtake.outtake(flywheelSpeed);
      outtake.pivot(pivotAngle / 360 * OuttakeConstants.gearReductionEncoderToMotor);
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
