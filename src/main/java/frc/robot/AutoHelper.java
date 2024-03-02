package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.centerline.CenterLineManager.CenterLineScoringStrategy;

public class AutoHelper {

  public enum Auto {
    None,
    LeaveSourceCorner,
    OneNoteCenter,
    OneNoteRight,
    OneNoteLeft,
    TwoNoteCenter,
    ThreeNoteCenter,
    FourNoteCenter
  }

  public static void configAutoChooser(SendableChooser<Auto> chooser) {
    chooser.setDefaultOption(Auto.None.toString(), Auto.None);
    for (Auto auto : Auto.values()) {
      if (auto != Auto.None) {
        chooser.addOption(auto.toString(), auto);
      }
    }
  }

  public static String getPathPlannerAutoName(Auto auto) {
    switch (auto) {
      case LeaveSourceCorner:
        return "LeaveInTheMiddleOfNowhere";
      default:
        return "None";
    }
  }

  public static CenterLineScoringStrategy getCenterLineScoringStrategy(Auto auto) {
    switch (auto) {
      default:
        return CenterLineScoringStrategy.DoNothing;
    }
  }
}
