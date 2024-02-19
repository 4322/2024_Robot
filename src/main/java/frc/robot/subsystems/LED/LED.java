package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  public LedIO io;
  private LEDState currentState = LEDState.idle;

  public enum LEDState {
    // priority 1 runs for undefined amount of time so keep time at 0
    idle,
    autoNoteAcq,
    notZeroed,
    zeroed,
    shooterReady,
    gamePieceDetected;
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
}
