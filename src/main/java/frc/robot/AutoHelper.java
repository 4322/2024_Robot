package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.centerline.CenterLineManager.CenterLineScoringStrategy;

public class AutoHelper {

  public enum Auto {
    None,
    LeaveSourceCorner,
    OneNoteCenter,
    OneNoteAmp,
    OneNoteSource,
    TwoNoteCenterStraightBack,
    TwoNoteCenterToAmp,
    TwoNoteCenterToSource,
    TwoNoteAmp,
    TwoNoteSource,
    ThreeNoteCenterToAmp,
    ThreeNoteCenterToSource,
    ThreeNoteAmp,
    ThreeNoteSource,
    FourNoteCenterToAmp,
    FourNoteCenterToSource,
    FourNoteAmp,
    FourNoteSource,
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
      case OneNoteCenter:
        return "OneNoteCenter";
      case OneNoteAmp:
        return "OneNoteAmp";
      case OneNoteSource:
        return "OneNoteSource";
      case TwoNoteCenterStraightBack:
        return "TwoNoteCenterStraightBack";
      case TwoNoteCenterToAmp:
        return "TwoNoteCenterToAmp";
      case TwoNoteCenterToSource:
        return "TwoNoteCenterToSource";
      case ThreeNoteCenterToAmp:
        return "ThreeNoteCenterToAmp";
      case ThreeNoteCenterToSource:
        return "ThreeNoteCenterToSource";
      case FourNoteCenterToAmp:
        return "FourNoteCenterToAmp";
      case FourNoteCenterToSource:
        return "FourNoteCenterToSource";
      case None:
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
