package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  public LedIO io;
  private LEDState currentState = LEDState.idle;
  private LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();
  private final int firstLed;
  private final int numLeds;

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

    if (Constants.debug) {
      // reserve a few LEDs for debug feedback
      firstLed = 4;
      numLeds = Constants.LED.totalLEDs - 4;
    } else {
      firstLed = 0;
      numLeds = Constants.LED.totalLEDs;
    }
  }

  @Override
  public void periodic() {
    // initial check
    if (Constants.ledEnabled) {
      io.updateInputs(inputs);
      Logger.processInputs("LED/", inputs); 
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
          io.setLED(0, 255, 255, 0, Constants.LED.totalLEDs);
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

  public void setDebugLed(int red, int green, int blue, int Led) {
    if (Constants.debug) {
      // last caller wins
      io.setLED(red, green, blue, Led, 1);
    }
  }

  public LEDState getLEDState() {
    return currentState;
  }
}
