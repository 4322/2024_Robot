package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ClimberSlowRetractOverride extends Command {
  private Climber climber;
  private OperatorXboxControllerRumble xBoxRumble;

  public ClimberSlowRetractOverride() {
    xBoxRumble = new OperatorXboxControllerRumble();
    climber = Climber.getInstance();
    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.slowRetractOverride();
  }

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
