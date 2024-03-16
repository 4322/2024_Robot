package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.climber.Climber;

public class ClimberExtend extends Command {
  private Climber climber;
  private OperatorXboxControllerRumble xBoxRumble;

  public ClimberExtend() {
    climber = Climber.getInstance();
    xBoxRumble = new OperatorXboxControllerRumble();
    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.extend();
    if (climber.isFullyExtended()) {
      CommandScheduler.getInstance().schedule(xBoxRumble);
    }
  }

  @Override
  public boolean isFinished() {
    return climber.isFullyExtended();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopClimb();
  }
}
