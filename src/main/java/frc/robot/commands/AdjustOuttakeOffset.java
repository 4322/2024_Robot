package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
public class AdjustOuttakeOffset extends InstantCommand{
     double offsetChange;
     public AdjustOuttakeOffset(double _offsetChange)
     {
        offsetChange = _offsetChange;
     }
     @Override
    public void initialize() {
            if (RobotCoordinator.getInstance().canPivot()) {
                Outtake.getInstance().adjustOffset(offsetChange);
            }
    }

  @Override
  public void end(boolean interrupted) {}
}
