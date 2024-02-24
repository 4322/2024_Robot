package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;

public class OuttakeStop extends InstantCommand {
  private Outtake outtake;
  public OuttakeStop() {
    outtake = Outtake.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outtake.stopOuttake();
    outtake.stopPivot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
