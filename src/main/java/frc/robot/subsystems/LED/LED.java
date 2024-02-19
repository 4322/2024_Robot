package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  public LedIO io;
  private LEDState currentState = LEDState.flashing;

  public enum LEDState {
    // priority 1 runs for undefined amount of time so keep time at 0
    rainbow,
    flashing,
    red,
    green,
    blue,
    purple;
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
    switch (currentState) {
      case rainbow:
        io.rainbowAnimate(1, 0.3, 20, 0);
        break;
      case flashing:
        io.flashAnimate(255, 0, 0, 0.6, 20, 6);
        break;
      case red:
        io.setLED(255, 0, 0, 0, Constants.LED.totalLEDs);
        break;
      case green:
        io.setLED(0, 255, 0, 0, Constants.LED.totalLEDs);
        break;
      case blue:
        io.setLED(0, 0, 255, 0, Constants.LED.totalLEDs);
        break;
      case purple:
        io.setLED(238, 130, 238, 0, Constants.LED.totalLEDs);
        break;
    }
  }

  public void setLEDState(LEDState state) {
    if (currentState != state) {
      currentState = state;
    }
  }

  public LEDState getLEDState() {
    return currentState;
  }
}
