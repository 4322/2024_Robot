package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.RobotCoordinator.RobotStates;

public class XboxControllerRumble extends Command {
  private final CommandXboxController xbox;

  private Timer rumbleTimer = new Timer();
  private boolean hasRumbled;

  public XboxControllerRumble(CommandXboxController xboxController) {

    xbox = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().noteInTunnel() && !hasRumbled 
          && RobotCoordinator.getInstance().getRobotState() == RobotStates.noteSecured) {
        rumbleTimer.start();
        xbox.getHID().setRumble(RumbleType.kBothRumble, 1);
    }
  }

  @Override
  public boolean isFinished() {
    return rumbleTimer.hasElapsed(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xbox.getHID().setRumble(RumbleType.kBothRumble, 0);
    rumbleTimer.stop();
    rumbleTimer.reset();
    hasRumbled = false;
  }
}

