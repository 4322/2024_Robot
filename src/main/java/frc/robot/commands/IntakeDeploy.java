package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.RobotCoordinator.RobotStates;

public class IntakeDeploy extends Command {
  private boolean initialDeploy;
  private RobotCoordinator coordinator;

  public IntakeDeploy() {
    coordinator = RobotCoordinator.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialDeploy = false;
  }

  @Override
  public void execute() {
    if (!initialDeploy && coordinator.getRobotState() != RobotStates.noteObtained) {
      coordinator.setRobotState(RobotStates.deploy);
      initialDeploy = true;
    }
    else if (coordinator.getRobotState() == RobotStates.noteSecured && coordinator.isAcrossCenterLine()) {
      coordinator.setRobotState(RobotStates.deploy);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (coordinator.getRobotState() == RobotStates.noteObtained) {
      return;
    }
    else {
      coordinator.setRobotState(RobotStates.retract);
    }
  }
}
