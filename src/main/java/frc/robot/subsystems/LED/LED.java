package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  public LedIO io;
  private LEDState currentState = LEDState.idle;
  private Timer initTimer = new Timer();
  private Timer outtakePresetTimer = new Timer();
  private LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  private boolean presetHasChanged;

  public enum LEDState {
    notInitialized,
    initialized,
    idle,
    operatorPreset,
    noteFired,
    noteReadyToShoot,
    noteInRobot,
    brakeMode,
    coastMode;
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

      if (DriverStation.isEnabled()) {
        if (RobotCoordinator.getInstance().noteIsShot()) {
          setLEDState(LEDState.noteFired);
        } else if (RobotCoordinator.getInstance().outtakePresetChanged() 
          || (!outtakePresetTimer.hasElapsed(0.1) && presetHasChanged)) {
          outtakePresetTimer.start();
          presetHasChanged = true;
          if (outtakePresetTimer.hasElapsed(0.1)) {
            outtakePresetTimer.stop();
            outtakePresetTimer.reset();
            presetHasChanged = false;
          }
          setLEDState(LEDState.operatorPreset);
        } else if (RobotCoordinator.getInstance().canShoot()
            && RobotCoordinator.getInstance().noteInFiringPosition()) {
          setLEDState(LEDState.noteReadyToShoot);
        } else if (RobotCoordinator.getInstance().noteInRobot()) {
          setLEDState(LEDState.noteInRobot);
        } else {
          setLEDState(LEDState.idle);
        }
      } else {
        if (!RobotCoordinator.getInstance().isInitialized()) {
          setLEDState(LEDState.notInitialized);
        } else if (RobotCoordinator.getInstance().isInitialized()) {
          setLEDState(LEDState.initialized);
        } else if (RobotCoordinator.getInstance().deployInCoast()
          && RobotCoordinator.getInstance().pivotInCoast()) {
          setLEDState(LEDState.coastMode);
        } else if (!RobotCoordinator.getInstance().deployInCoast()
          && !RobotCoordinator.getInstance().pivotInCoast()) {
          setLEDState(LEDState.brakeMode);
        } else {
          setLEDState(LEDState.idle);
        }
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
          // red
          io.setLED(255, 0, 0, 0, Constants.LED.totalLEDs);
          break;
        case initialized:
          // green
          io.setLED(0, 255, 0, 0, Constants.LED.totalLEDs);
          break;
        case idle:
          // flame
          io.fireAnimate(1, 0.5, Constants.LED.totalLEDs, 0.5, 0.5, false, 0);
          break;
        case noteInRobot:
          // blue
          io.setLED(0, 0, 255, 0, Constants.LED.totalLEDs);
          break;
        case noteFired:
          // orange
          io.setLED(255, 165, 0, 0, Constants.LED.totalLEDs);
          break;
        case noteReadyToShoot:
          // green
          io.setLED(0, 255, 0, 0, Constants.LED.totalLEDs);
          break;
        case brakeMode:
          // orange
          io.setLED(255, 165, 0, 0, Constants.LED.totalLEDs);
          break;
        case coastMode:
          // green
          io.setLED(0, 255, 0, 0, Constants.LED.totalLEDs);
          break;
        case operatorPreset:
          // white
          io.setLED(255, 255, 255, 0, Constants.LED.totalLEDs);
          break;
      }
    }
  }

  public LEDState getLEDState() {
    return currentState;
  }
}
