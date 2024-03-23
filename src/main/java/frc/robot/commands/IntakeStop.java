package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.IntakeManual.IntakeManual;
import frc.robot.commands.IntakeManual.IntakeManual.IntakeStates;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;

public class IntakeStop extends InstantCommand {

  public IntakeStop() {
    addRequirements(Intake.getInstance());
  }

  @Override
  public void initialize() {
    Intake.getInstance().stopDeployer();
    Intake.getInstance().stopFeeder();
  }

  @Override
  public void end(boolean interrupted) {
    if (RobotCoordinator.getInstance().isIntakeDeployed()
        || RobotCoordinator.getInstance().noteInFiringPosition()) {
      IntakeManual.setIntakeState(IntakeStates.notePastIntake);
    } else if (RobotCoordinator.getInstance().isIntakeRetracted()) {
      IntakeManual.setIntakeState(IntakeStates.retracted);
    } else if (RobotCoordinator.getInstance().noteEnteringIntake()) {
      IntakeManual.setIntakeState(IntakeStates.noteObtained);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
