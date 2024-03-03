package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.outtake.Outtake;

public class ClimberExtend extends Command{
    private Climber climber;
    private Outtake outtake;
    private OperatorXboxControllerRumble xBoxRumble;
    public ClimberExtend()
    {
        climber = Climber.getInstance();
        outtake = Outtake.getInstance();
        xBoxRumble = new OperatorXboxControllerRumble();
        addRequirements(climber, outtake);
    }
    @Override
    public void execute()
    {
       climber.extend();
    }
    @Override
    public boolean isFinished()
    {
        return false;
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