package frc.robot.commands.CenterLine;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CenterLine.stateMachine.CLSM;
import frc.robot.subsystems.drive.Drive;

public class ScoreCenterLine extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive drive;

  private final CLSM machine;

  public enum ScoringStrategy {
    OneToFive,
    DoNothing,
  }

  public ScoreCenterLine(Drive drivesubsystem, ScoringStrategy strategy) {
    drive = drivesubsystem;
    machine = new CLSM(strategy);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
