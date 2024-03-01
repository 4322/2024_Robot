package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotCoordinator;

// pressed when intake and outtake are in starting config
// can only be pressed once after bootup
public class AtHome extends InstantCommand {
  public AtHome() {}

  @Override
  public void initialize() {
    if (DriverStation.isDisabled()) {
      RobotCoordinator.getInstance().setInitAbsEncoderPressed(true);
      DriverStation.reportWarning("Initialized motor positions", false);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
