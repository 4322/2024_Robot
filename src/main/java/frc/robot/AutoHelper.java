package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.centerline.CenterLineManager.CenterLineScoringStrategy;

public class AutoHelper {

  public enum Auto {
    None,
    SmartSixNoteTop,
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
      case SmartSixNoteTop:
        return "3NoteToTopShoot";
      default:
        return "None";
    }
  }

  public static CenterLineScoringStrategy getCenterLineScoringStrategy(Auto auto) {
    switch (auto) {
      case SmartSixNoteTop:
        return CenterLineScoringStrategy.OneToFive;
      default:
        return CenterLineScoringStrategy.DoNothing;
    }
  }
}
