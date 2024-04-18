package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooting.FiringSolution;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;

public class AutoSetOuttakeAdjust extends InstantCommand {
  private Outtake outtake;
  private double flywheelSpeed;
  private double pivotRotations;

  public AutoSetOuttakeAdjust(FiringSolution solution) {
    outtake = Outtake.getInstance();
    this.flywheelSpeed = solution.getFlywheelSpeed();
    this.pivotRotations = solution.getShotRotations();

    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    if (RobotCoordinator.getInstance().canPivot()) {
      // Call stop instead of setting to 0 rps in order to force 
      // brake mode and avoid tunnel feed command issue explained in stop method.
      if (flywheelSpeed == 0) { 
        outtake.stopOuttake();
      }
      else {
        outtake.outtake(flywheelSpeed);
      }
      outtake.pivot(pivotRotations);
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
