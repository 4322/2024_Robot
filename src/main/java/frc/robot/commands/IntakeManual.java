package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class IntakeManual extends Command {
  public enum IntakeStates {
    retracted,
    deploying,
    feeding,
    noteObtained,
    notePastIntake,
    retracting;
  }

  private IntakeStates intakeState = IntakeStates.retracted;
  private Intake intake;
  private AutoAcquireNote autoAcquireNote;

  public IntakeManual() {
    intake = Intake.getInstance();
    autoAcquireNote = new AutoAcquireNote();

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intakeState = IntakeStates.retracting;
  }

  @Override
  public void execute() {
    RobotCoordinator coordinator = RobotCoordinator.getInstance();
    switch (intakeState) {
      case retracted:
        if (coordinator.getIntakeButtonPressed()) {
          intakeState = IntakeStates.deploying;
        }
        break;
      case deploying:
        if (coordinator.canDeploy()) {
          intake.deploy();
          intake.intake();  // maybe jump to feeding state instead?
        }
        if (!coordinator.getIntakeButtonPressed()) {
          intakeState = IntakeStates.retracting;
        } else if (coordinator.isIntakeDeployed() && !coordinator.noteInRobot()) {
          intakeState = IntakeStates.feeding;
        } else if (coordinator.noteInRobot() && coordinator.isIntakeDeployed()) {
          intakeState = IntakeStates.notePastIntake;
        }
        break;
      case feeding:
        if (coordinator.isIntakeDeployed()) {
          intake.intake();
        }
        if (!coordinator.getIntakeButtonPressed()) {
          intakeState = IntakeStates.retracting;
        } else if (coordinator.noteEnteringIntake()) {
          intakeState = IntakeStates.noteObtained;
        } else if (Constants.autoAcquireNoteEnabled) {
          if (coordinator.getAutoIntakeButtonPressed()
              && coordinator.noteInVision()
              && !autoAcquireNote.isScheduled()) {
            CommandScheduler.getInstance().schedule(autoAcquireNote);
          }
        }
        break;
      case noteObtained:
        if (coordinator.isIntakeDeployed()) {
          intake.intake();
        }
        if (autoAcquireNote.isScheduled()) {
          autoAcquireNote.cancel();
        }
        if (!coordinator.noteEnteringIntake()) {
          intakeState = IntakeStates.notePastIntake;
        }
        break;
      case notePastIntake:
        intake.stopFeeder();
        if (!coordinator.getIntakeButtonPressed()) {
          intakeState = IntakeStates.retracting;
        } else if (!coordinator.noteInRobot()) {
          intakeState = IntakeStates.feeding;
        }
        break;
      case retracting:
        intake.stopFeeder();
        if (coordinator.canRetract()) {
          intake.retract();
        }
        if (autoAcquireNote.isScheduled()) {
          autoAcquireNote.cancel();
        }
        if (coordinator.getIntakeButtonPressed()) {
          intakeState = IntakeStates.deploying;
        } else if (coordinator.isIntakeRetracted()) {
          intakeState = IntakeStates.retracted;
        }
        break;
    }
    Logger.recordOutput("Intake/ManualState/", intakeState.toString());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
