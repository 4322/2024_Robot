package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

public class IntakeStop extends InstantCommand {

  public IntakeStop() {
    addRequirements(Intake.getInstance());
  }

  @Override
  public void initialize() {
    Intake.getInstance().stopDeployer();
    Intake.getInstance().stopFeeder();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
