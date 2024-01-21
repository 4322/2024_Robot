package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase implements IntakeInterface {
  
  private 

  public Intake() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.intakeEnabled) {
          
        }
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }
  }

  @Override
  public void periodic() {

  }

}
