package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtakePivot.OuttakePivot;

public class OuttakeResetPivot extends Command{
    private final OuttakePivot outtakePivot;
    public OuttakeResetPivot() {
        outtakePivot = OuttakePivot.getInstance();
        addRequirements(outtakePivot);
    }
    @Override
    public void initialize()
    {
        outtakePivot.resetPivot();
    }
    @Override
    public void end(boolean interrupted)
    {
        outtakePivot.stopPivot();
    }
}
