package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.climber.Climber;

public class ClimberSlowRetractOverride extends Command{
    private Climber climber;
    private OperatorXboxControllerRumble xBoxRumble;
    public ClimberSlowRetractOverride()
    {
        xBoxRumble = new OperatorXboxControllerRumble();
        climber = Climber.getInstance();
        addRequirements(climber);
    }
    @Override
    public void execute()
    {
        climber.slowRetractOverride();
    }
    @Override
    public boolean isFinished()
    {
        return false;
    } 
    @Override
    public void end(boolean interrupted)
    {
        if(climber.isFullyRetracted())
        {
            CommandScheduler.getInstance().schedule(xBoxRumble);
        }
        climber.stopClimb();
    }
}
