package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtakePivot.OuttakePivot;

public class PivotToAngle extends Command {
 
  OuttakePivot outtakePivot;
  double angle;
  public PivotToAngle(double angle)
  {
    outtakePivot = OuttakePivot.getInstance();
    addRequirements(outtakePivot);
    this.angle = angle;
  }
  @Override
  public void initialize() {
    outtakePivot.pivot(angle/360);
  }
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    outtakePivot.stopPivot();
  }
}
