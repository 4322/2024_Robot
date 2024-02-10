package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

//unsure if this will be necessary
public class OuttakeStop extends Command{
    Outtake outtake;
    OuttakeStop(Outtake outtakeSubsystem)
    {
        outtake = outtakeSubsystem;
        addRequirements(outtake);
    }
    @Override
    public void initialize() {
        outtake.stopOuttake();
    }
    @Override
    public void end(boolean interrupted) {
        
    }
}
