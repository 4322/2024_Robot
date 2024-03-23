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
    retracting,
    climb;
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
        if (coordinator.getIntakeButtonPressed()) {
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
        if (!coordinator.noteEnteringIntake()) {
          CommandScheduler.getInstance().schedule(xBoxRumble);
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
        if (coordinator.getIntakeButtonPressed()) {
          intakeState = IntakeStates.deploying;
        } else if (coordinator.isIntakeRetracted()) {
          intakeState = IntakeStates.retracted;
        }
        break;
      case climb:
        // go to climb state regardless of previous intake state
        intake.deployClimbPosition();
        if (coordinator.getIntakeButtonPressed()) {
          intakeState = IntakeStates.deploying;
        }
        break;
    }
    Logger.recordOutput("Intake/ManualState/", intakeState.toString());
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
