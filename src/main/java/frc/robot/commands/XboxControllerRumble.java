package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotCoordinator;

public class XboxControllerRumble extends Command {

  private Timer rumbleTimer = new Timer();
  private boolean hasRumbled;

  public XboxControllerRumble() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().noteInFiringPosition()
        && !hasRumbled
        && RobotCoordinator.getInstance().getIntakeState()
            == IntakeManual.IntakeStates.notePastIntake) {
      rumbleTimer.start();
      RobotContainer.driveXbox.getHID().setRumble(RumbleType.kBothRumble, 1);
    }
  }

  @Override
  public boolean isFinished() {
    return rumbleTimer.hasElapsed(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
    rumbleTimer.stop();
    rumbleTimer.reset();
    hasRumbled = false;
  }
}
