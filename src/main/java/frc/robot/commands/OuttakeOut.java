package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;
//may change this to take firing solution
public class OuttakeOut extends Command {
    Outtake outtake;
    double outtakeRPM;
    public OuttakeOut(Outtake outtakeSubsystem, double targetOuttakeRPM)
    {
        outtake = outtakeSubsystem;
        outtakeRPM = targetOuttakeRPM;
        addRequirements(outtake);
    }
    @Override
    public void initialize() {
        outtake.outtake(outtakeRPM);
    }
    @Override
    public void end(boolean interrupted) {
        outtake.stopOuttake();;
    }
}