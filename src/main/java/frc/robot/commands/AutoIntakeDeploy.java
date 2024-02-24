package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;

public class AutoIntakeDeploy extends InstantCommand {

    public AutoIntakeDeploy() {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        if (RobotCoordinator.getInstance().canDeploy()) {
            Intake.getInstance().deploy();
        }  
    }

    @Override
    public void end(boolean interrupted) {}
}
