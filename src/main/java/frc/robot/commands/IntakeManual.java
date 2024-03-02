package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;

public class IntakeManual extends Command {
  public enum IntakeStates {
    retracted,
    deploying,
    feeding,
    noteObtained,
    notePastIntake,
    retracting;
  }

  private static IntakeStates intakeState = IntakeStates.retracted;
  ;
  private Intake intake;
  private AutoAcquireNote autoAcquireNote;
  private XboxControllerRumble xBoxRumble;

  public IntakeManual() {
    intake = Intake.getInstance();
    autoAcquireNote = new AutoAcquireNote();
    xBoxRumble = new XboxControllerRumble();

    addRequirements(intake);
  }

  @Override
  public void execute() {
    RobotCoordinator coordinator = RobotCoordinator.getInstance();
    switch (intakeState) {
      case retracted:
        if (coordinator.getIntakeButtonPressed()
            && (coordinator.onOurSideOfField() || !coordinator.noteInRobot())) {
          intakeState = IntakeStates.deploying;
        }
        break;
      case deploying:
        if (coordinator.canDeploy()) {
          intake.deploy();
        }
        if (!coordinator.getIntakeButtonPressed()) {
          intakeState = IntakeStates.retracting;
        } else if (coordinator.intakeIsDeployed() && !coordinator.noteInRobot()) {
          intakeState = IntakeStates.feeding;
        } else if (coordinator.noteInRobot() && coordinator.intakeIsDeployed()) {
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
        } else if (Constants.autoAcquireNoteEnabled
            && coordinator.getAutoIntakeButtonPressed()
            && coordinator.noteInVision()) {
          if (!autoAcquireNote.isScheduled()) {
            CommandScheduler.getInstance().schedule(autoAcquireNote);
          }
        }
        break;
      case noteObtained:
        if (coordinator.isIntakeDeployed()) {
          intake.intake();
        }
        if (!coordinator.noteEnteringIntake()) {
          CommandScheduler.getInstance().schedule(xBoxRumble);
          intakeState = IntakeStates.notePastIntake;
        }
        break;
      case notePastIntake:
        intake.stopFeeder();
        if (!coordinator.onOurSideOfField() || !coordinator.getIntakeButtonPressed()) {
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
        if (coordinator.getIntakeButtonPressed()
            && (coordinator.onOurSideOfField() || !coordinator.noteInRobot())) {
          intakeState = IntakeStates.deploying;
        } else if (coordinator.isIntakeRetracted()) {
          intakeState = IntakeStates.retracted;
        }
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static IntakeStates getIntakeState() {
    return intakeState;
  }

  public static void setIntakeState(IntakeStates newState) {
    intakeState = newState;
  }
}
