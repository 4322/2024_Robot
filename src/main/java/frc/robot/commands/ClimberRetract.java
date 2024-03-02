package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.climber.Climber;

public class ClimberRetract extends Command{
    private Climber climber;
    private OperatorXboxControllerRumble xBoxRumble;
    public ClimberRetract()
    {
        xBoxRumble = new OperatorXboxControllerRumble();
        climber = Climber.getInstance();
        addRequirements(climber);
    }
    @Override
    public void execute()
    {
        climber.retract();
    }
    @Override
    public boolean isFinished()
    {
        return (climber.isFullyRetracted());
    } 
    @Override
    public void end(boolean interrupted)
    {
        if(climber.isFullyExtended())
        {
            CommandScheduler.getInstance().schedule(xBoxRumble);
        }
        climber.stopClimb();
    }
}
