package frc.robot.subsystems.drive.RobotChooser;

import frc.robot.Constants;

public class RobotChooser {

  private static RobotChooser chooser;
  private RobotChooserInterface robotConstants;

  public static RobotChooser getInstance() {
    if (chooser == null) {
      chooser = new RobotChooser();
    }
    return chooser;
  }

  private RobotChooser() {
    switch (Constants.currentRobot) {
      case NEMO:
        robotConstants = new NemoConstants();
        break;
      case CRUSH:
        robotConstants = new CrushConstants();
        break;
    }
  }

  public RobotChooserInterface getConstants() {
    return robotConstants;
  }
}
