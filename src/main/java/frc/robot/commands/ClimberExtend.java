package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.outtake.Outtake;

public class ClimberExtend extends Command{
    private Climber climber;
    private Outtake outtake;
    public ClimberExtend()
    {
        climber = Climber.getInstance();
        outtake = Outtake.getInstance();
        addRequirements(climber, outtake);
    }
    @Override
    public void execute()
    {
        RobotCoordinator coordinator = RobotCoordinator.getInstance();
        outtake.pivot(ClimberConstants.climbingPivotRotations); //unsure if sequential command group is necessary- if so, I'll revise.
        if(coordinator.pivotAtPosition())
        {
            climber.extend();
        }
    }
    @Override
    public boolean isFinished()
    {
        return (climber.isFullyExtended());
    } 
    @Override
    public void end(boolean interrupted)
    {
        climber.stopClimb();
    }
}
