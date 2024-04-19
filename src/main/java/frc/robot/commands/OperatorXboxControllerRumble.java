package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class OperatorXboxControllerRumble extends Command {
  private Timer rumbleTimer = new Timer();
  private double rumbleTime;

  public OperatorXboxControllerRumble(double time) {
    rumbleTime = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    rumbleTimer.start();
    RobotContainer.operatorXbox.getHID().setRumble(RumbleType.kBothRumble, 1);
  }

  @Override
  public boolean isFinished() {
    return rumbleTimer.hasElapsed(rumbleTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.operatorXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
    rumbleTimer.stop();
    rumbleTimer.reset();
  }
}
