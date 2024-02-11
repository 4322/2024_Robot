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
          NoteTracker.noteTrackerSensorsIO = new DistanceSensorIOReal();
          break;
        case SIM:
          break;
        case REPLAY: 
          break;
      }
      
      if (NoteTracker.noteTrackerSensorsIO == null) {
        NoteTracker.noteTrackerSensorsIO = new DistanceSensorIO() {};
      }
  }

  @Override
  public void periodic() {
    NoteTracker.noteTrackerSensorsIO.updateInputs(NoteTracker.inputs);
  }

  public boolean canIntake() {
    return intakeDeployer.isAtPosition() && intakeDeployer.isDeployed();
  }

  public boolean canDeploy() {
    return intakeDeployer.isInitialized() && !intakeDeployer.isDeployed();
  }

  public boolean canShoot() {
    return outtake.isFlyWheelUpToSpeed() && outtakePivot.isAtPosition() && NoteTracker.inFiringPos();
  }

  public boolean canPivot() {
    return outtakePivot.isInitialized();
  }

  public boolean intakingNote() {
      return NoteTracker.inputs.intakeBeamBreak && intake.getState() == IntakeStates.INTAKING;
  }

  private static class NoteTracker {
    private static DistanceSensorIO noteTrackerSensorsIO;
    private static DistanceSensorIOInputsAutoLogged inputs = new DistanceSensorIOInputsAutoLogged();

    public static boolean inFiringPos() {
      return OrangeMath.equalToTwoDecimal(inputs.tunnelDistance, Constants.TunnelConstants.noteToSensorDistMeters);
    }

    public static boolean hasPassedIntake() {
      // Used for auto because check is faster
      return inputs.intakeBeamBreak || inputs.tunnelBeamBreak;
    }
  }
}
