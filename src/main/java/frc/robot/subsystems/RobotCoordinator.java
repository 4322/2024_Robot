package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.intakeDeployer.IntakeDeployer;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtakePivot.OuttakePivot;
import frc.utility.OrangeMath;

public class RobotCoordinator extends SubsystemBase {

  private Intake intake;
  private Outtake outtake;
  private IntakeDeployer intakeDeployer;
  private OuttakePivot outtakePivot;

  private static DistanceSensorIO noteTrackerSensorsIO;
  private static DistanceSensorIOInputsAutoLogged inputs = new DistanceSensorIOInputsAutoLogged();

  private static RobotCoordinator robotCoordinator;

  public static RobotCoordinator getInstance() {
    if (robotCoordinator == null) {
      robotCoordinator = new RobotCoordinator();
    }
    return robotCoordinator;
  }

  private RobotCoordinator() {
    intake = Intake.getInstance();
    outtake = Outtake.getInstance();
    intakeDeployer = IntakeDeployer.getInstance();
    outtakePivot = OuttakePivot.getInstance();
    switch (Constants.currentMode) {
      case REAL:
        noteTrackerSensorsIO = new DistanceSensorIOReal();
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }

    if (noteTrackerSensorsIO == null) {
      noteTrackerSensorsIO = new DistanceSensorIO() {};
    }
  }

  @Override
  public void periodic() {
    noteTrackerSensorsIO.updateInputs(inputs);
  }

  public boolean canIntake() {
    return intakeDeployer.isAtPosition() && intakeDeployer.isDeployed();
  }

  public boolean canDeploy() {
    return intakeDeployer.isInitialized() && !intakeDeployer.isDeployed();
  }

  public boolean canShoot() {
    return outtake.isFlyWheelUpToSpeed()
        && outtakePivot.isAtPosition()
        && noteInFiringPos();
  }

  public boolean canPivot() {
    return outtakePivot.isInitialized();
  }

  public boolean intakingNote() {
    return inputs.intakeBeamBreak && intake.getState() == IntakeStates.INTAKING;
  }

  public boolean noteInRobot() {
    return !inputs.intakeBeamBreak && inputs.tunnelBeamBreak;
  }

  public boolean noteInFiringPos() {
    return OrangeMath.equalToTwoDecimal(
      inputs.tunnelDistance, Constants.TunnelConstants.noteToSensorDistMeters);
  }

}
