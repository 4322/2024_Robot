package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinatorInterface;
import java.util.ArrayList;

public class LED extends SubsystemBase {
  public LedIO io;
  private ArrayList<LEDState> queuedStates = new ArrayList<LEDState>();
  private ArrayList<LEDState> prevQueuedStates = new ArrayList<LEDState>();
  private RobotCoordinatorInterface robotCoordinator;
  private LEDState currentState = LEDState.idle;
  private Timer stateTimer = new Timer();

  public enum LEDState {
    // priority 1 runs for undefined amount of time so keep time at 0
    idle(1, 0),
    autoNoteAcq(1, 0),
    notZeroed(1, 0),
    zeroed(3, 0.5),
    shooterReady(3, 0.5),
    gamePieceDetected(3, 0.5);

    public int priorityNum;
    public double timeSec;

    LEDState(int priority, double time) {
      priorityNum = priority;
      timeSec = time;
    }
  }

  public LED() {
    switch (Constants.currentMode) {
      case REAL:
        io = new LedIOReal();
        break;

      case SIM:
        break;

      case REPLAY:
        break;
    }

    if (io == null) {
      io = new LedIO() {};
    }
  }

  @Override
  public void periodic() {
    io.rainbowAnimate(1, 0.3, 5, 0);
    io.flashAnimate(255, 0, 0, 0.6, 5, 6);
    io.setLED(0, 255, 0, 11, 5);

    determineStates();

    switch (prioritizeSingleState()) {
      case idle:
        io.rainbowAnimate(1, 0.3, 20, 0);
        break;
      case notZeroed:
        io.flashAnimate(255, 0, 0, 0.6, 20, 6);
        break;
      case zeroed:
        io.setLED(0, 255, 0, 0, Constants.LED.totalLEDs);
        break;
      case autoNoteAcq:
        break;
      case shooterReady:
        break;
      case gamePieceDetected:
        break;
    }
  }

  private void determineStates() {
    // TODO: continue for all defined LED states

    if (robotCoordinator.canShoot()) {
      queuedStates.add(LEDState.shooterReady);
    }
    if (robotCoordinator.hasNote()) {
      queuedStates.add(LEDState.gamePieceDetected);
    }
    if (robotCoordinator.canDeploy() && robotCoordinator.canPivot()) {
      queuedStates.add(LEDState.zeroed);
    }
    if (!robotCoordinator.canDeploy() && !robotCoordinator.canPivot()) {
      queuedStates.add(LEDState.notZeroed);
    }

    // timer check must be under all state checks
    // accounts for time-based LED states
    if (currentState.priorityNum != 1 && stateTimer.hasElapsed(currentState.timeSec)) {
      queuedStates.remove(currentState);
      // set to default 1st priority state so it can either be overriden or maintained when checked
      for (LEDState state : queuedStates) {
        if (state.priorityNum == 1) {
          currentState = state;
        }
      }
    }
  }

  private LEDState prioritizeSingleState() {
    // only check if there is new state in queue
    if (!queuedStates.equals(prevQueuedStates)) {
      prevQueuedStates.clear();
      prevQueuedStates.addAll(queuedStates);
      for (LEDState state : queuedStates) {
        if (currentState.priorityNum != 3 && state.priorityNum >= currentState.priorityNum) {
          currentState = state;
          stateTimer.stop();
          stateTimer.reset();
        }
      }
    }
    queuedStates.clear();
    stateTimer.start(); // ignored if already running
    return currentState;
  }
}
