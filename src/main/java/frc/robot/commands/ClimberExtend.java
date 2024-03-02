package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.climber.Climber;

public class ClimberExtend extends Command{
    private Climber climber;
    public ClimberExtend()
    {
        climber = Climber.getInstance();
        addRequirements(climber);
    }
    @Override
    public void execute()
    {
        RobotCoordinator coordinator = RobotCoordinator.getInstance();
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
