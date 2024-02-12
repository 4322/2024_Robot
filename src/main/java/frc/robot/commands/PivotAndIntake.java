package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeDeployer.IntakeDeployer;

public class PivotAndIntake extends Command {
  private final Intake intake;
  private final IntakeDeployer intakeDeployer;
  private final RobotCoordinator coordinator;

  private boolean deployed;

  public PivotAndIntake() {
    intake = Intake.getInstance();
    intakeDeployer = IntakeDeployer.getInstance();
    coordinator = RobotCoordinator.getInstance();

    deployed = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, intakeDeployer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (coordinator.canDeploy() && !deployed) { // only deploy once to not break PID (just in case)
      deployed = true;
      intakeDeployer.deploy();
    }
    if (coordinator.canIntake() && !coordinator.noteInRobot()) {
      intake.intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
