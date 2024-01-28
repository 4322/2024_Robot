package frc.robot.commands.DriveAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveAuto.DriveAutoStateMachine.DriveAutoState;
import frc.robot.commands.DriveAuto.DriveAutoStateMachine.DriveAutoTrigger;
import frc.robot.subsystems.drive.Drive;

public class DriveAuto extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive drive;

  private final DriveAutoStateMachine stateMachine;

  public DriveAuto(Drive drivesubsystem) {
    drive = drivesubsystem;
    stateMachine = new DriveAutoStateMachine(DriveAutoState.DEFAULT);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetRotatePID();
  }

  @Override
  public void execute() {
    switch (stateMachine.getState()) {
      case DEFAULT:
        break;
      case NOTE:
        break;
      case NO_NOTE:
        break;
    }
  }

  public void updateStateMachine(DriveAutoTrigger trigger) {
    stateMachine.fire(trigger);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
