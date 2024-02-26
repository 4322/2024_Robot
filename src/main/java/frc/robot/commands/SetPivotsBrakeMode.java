package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

public class SetPivotsBrakeMode extends InstantCommand {
  public SetPivotsBrakeMode() {
    addRequirements(Intake.getInstance(), Outtake.getInstance());
  }

  @Override
  public void initialize() {
    if (DriverStation.isDisabled()) {
      Intake.getInstance().setDeployerBrakeMode();
      Outtake.getInstance().setPivotBrakeMode();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
