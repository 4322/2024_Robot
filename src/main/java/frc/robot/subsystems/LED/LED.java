package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color;
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
    outtakeAtFiringPosition,
    noteReadyToShoot,
    noteInRobot,
    alignedWithAmp,
    aligningWithAmp,
    alignedWithSource,
    aligningWithSource,
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

    if (Constants.autoRotateDebug) {
      // reserve a few LEDs for debug feedback
      firstLed = 0;
      numLeds = Constants.LED.totalLEDs - 4;
      io.setLED(Color.kBlack, Constants.LED.totalLEDs - 4, 4);
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
          io.setLED(Color.kRed, firstLed, numLeds);
          break;
        case initialized:
          io.setLED(Color.kIvory, firstLed, numLeds);
          break;
        case idle:
          io.setLED(Color.kViolet, firstLed, numLeds);
          break;
        case noteInRobot:
          io.setLED(Color.kBlack, firstLed, numLeds);
          break;
        case outtakeAtFiringPosition:
          io.setLED(Color.kRoyalBlue, numLeds, firstLed);
          break;
        case noteReadyToShoot:
          io.setLED(Color.kGreen, firstLed, numLeds);
          break;
        case brakeMode:
          io.setLED(Color.kOrange, firstLed, numLeds);
          break;
        case coastMode:
          io.setLED(Color.kDarkGreen, firstLed, numLeds);
          break;
        case operatorPreset:
          io.setLED(Color.kWhite, firstLed, numLeds);
          break;
        case aligningWithAmp:
          io.setLED(Color.kPink, firstLed, numLeds);
          break;
        case alignedWithAmp:
          io.setLED(Color.kBlue, firstLed, numLeds);
          break;
        case aligningWithSource:
          io.setLED(Color.kPink, firstLed, numLeds);
          break;
        case alignedWithSource:
          io.setLED(Color.kBlue, firstLed, numLeds);
          break;
      }
    }
  }

  public void setAutoRotateDebugLed(Color color, int led) {
    if (Constants.autoRotateDebug) {
      // last caller wins
      // first 8 LEDs are on the CANdle
      io.setLED(color, led, 1);
    }
  }

  public LEDState getLEDState() {
    return currentState;
  }
}
