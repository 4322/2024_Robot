package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ClimberSlowRetractOverride extends Command {
  private Climber climber;

  public ClimberSlowRetractOverride() {
    climber = Climber.getInstance();
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.slowRetractOverride();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopClimb();
    climber.zeroClimberAtCurrentPos();
  }
}
