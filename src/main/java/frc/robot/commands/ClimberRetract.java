package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ClimberRetract extends Command {
  private Climber climber;

  public ClimberRetract() {
    climber = Climber.getInstance();
    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.retract();
  }

  @Override
  public boolean isFinished() {
    return climber.isAtClimbRetractingThreshold();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopClimb();
  }
}
