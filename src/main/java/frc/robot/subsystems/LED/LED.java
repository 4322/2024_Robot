package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  public LedIO io;
  private LEDState currentState = LEDState.idle;
  private Timer initTimer = new Timer();
  private LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  public enum LEDState {
    notInitialized,
    initialized,
    idle,
    deployingIntake,
    noteInRobot,
    noteInFiringPos,
    noteReadyToShoot;
  }

  private static LED led;

  public static LED getInstance() {
    if (led == null) {
      led = new LED();
    }
    return led;
  }

  private LED() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.ledEnabled) {
          io = new LedIOReal();
        }
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
    // initial check
    if (Constants.ledEnabled) {
      io.updateInputs(inputs);
      Logger.processInputs("LED/", inputs);

      if (!RobotCoordinator.getInstance().getInitAbsEncoderPressed()
          && !RobotCoordinator.getInstance().isInitialized()) {
        setLEDState(LEDState.notInitialized);
      } else if (RobotCoordinator.getInstance().getInitAbsEncoderPressed()
          && RobotCoordinator.getInstance().isInitialized()
          && !initTimer.hasElapsed(1)) {
        initTimer.start();
        setLEDState(LEDState.initialized);
      } else if (RobotCoordinator.getInstance()
          .canShoot()) { // robot LED states listed from highest to lowest priority
        setLEDState(LEDState.noteReadyToShoot);
      } else if (RobotCoordinator.getInstance().noteInFiringPosition()) {
        setLEDState(LEDState.noteInFiringPos);
      } else if (RobotCoordinator.getInstance().noteInRobot()) {
        setLEDState(LEDState.noteInRobot);
      } else if (RobotCoordinator.getInstance().isIntakeDeployed()
          && RobotCoordinator.getInstance().isIntakeDeploying()) {
        setLEDState(LEDState.deployingIntake);
      } else {
        setLEDState(LEDState.idle);
      }
    }
  }

  public void setLEDState(LEDState state) {
    if (Constants.ledEnabled) {
      if (currentState != state) {
        Logger.recordOutput("LED/CurrentState/", state.toString());
        currentState = state;
        io.configBrightness(1); // reset brightness scalar
        io.clearAnimation(); // allows for other LED states to be set
      }
      switch (currentState) {
        case notInitialized:
          io.flashAnimate(255, 0, 0, 0.5, 0, Constants.LED.totalLEDs);
          break;
        case initialized:
          io.setLED(0, 255, 0, 0, Constants.LED.totalLEDs);
          break;
        case idle:
          io.setLED(0, 0, 255, 0, Constants.LED.totalLEDs);
          break;
        case deployingIntake:
          io.setLED(255, 0, 0, 0, Constants.LED.totalLEDs);
          break;
        case noteInRobot:
          io.setLED(255, 255, 255, 0, Constants.LED.totalLEDs);
          break;
        case noteInFiringPos:
          io.setLED(0, 255, 0, 0, Constants.LED.totalLEDs);
          break;
        case noteReadyToShoot:
          io.rainbowAnimate(1, 0.5, 0, Constants.LED.totalLEDs);
          break;
      }
    }
  }

  public LEDState getLEDState() {
    return currentState;
  }
}
