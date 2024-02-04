package frc.robot.subsystems.LED;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  public LedIO io;
  private ArrayList<LEDState> queuedStates = new ArrayList<LEDState>();
  private ArrayList<LEDState> prevQueuedStates = new ArrayList<LEDState>();

  public enum LEDState {
    idle(1), 
    autoNoteAcq(1), 
    shooterReady(3), 
    gamePieceDetected(3);

    public int priorityNum;

    LEDState(int priority) {
      priorityNum = priority;
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
  }

  public void determineStates() {
    
  }

  public void prioritizeStates() {

  }
  
}
