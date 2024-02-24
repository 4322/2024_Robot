package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;

public class AutoIntakeIn extends InstantCommand {
    
    public AutoIntakeIn() {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        if (RobotCoordinator.getInstance().isIntakeDeployed()) {
            Intake.getInstance().intake(); 
        }
    }

    @Override
    public void end(boolean interrupted) {}

}
