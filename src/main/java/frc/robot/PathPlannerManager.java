package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.RobotChooser.RobotChooser;
import frc.robot.subsystems.drive.RobotChooser.RobotChooserInterface;

class PathPlannerManager {
  private static RobotChooserInterface robotSpecificConstants =
      RobotChooser.getInstance().getConstants();

  public static void init(Drive driveSubsystem) {
    if (!AutoBuilder.isConfigured()) {
      HolonomicPathFollowerConfig holonomicConfig =
          new HolonomicPathFollowerConfig(
              new PIDConstants(
                  robotSpecificConstants.getAutoTrajectoryRotkP(),
                  robotSpecificConstants.getAutoTrajectoryRotkI(),
                  robotSpecificConstants.getAutoTrajectoryRotkD(),
                  robotSpecificConstants.getAutoTrajectoryRotkiZ()),
              new PIDConstants(
                  robotSpecificConstants.getAutoTrajectoryRotkP(),
                  robotSpecificConstants.getAutoTrajectoryRotkI(),
                  robotSpecificConstants.getAutoTrajectoryRotkD(),
                  robotSpecificConstants.getAutoTrajectoryRotkiZ()),
              DriveConstants.Auto.autoMaxModuleSpeedMetersPerSecond,
              DriveConstants.distWheelMetersR,
              new ReplanningConfig());

      AutoBuilder.configureHolonomic(
          driveSubsystem::getPose2d,
          driveSubsystem::resetOdometry,
          driveSubsystem::getChassisSpeeds,
          driveSubsystem::setModuleStatesFromChassisSpeeds,
          holonomicConfig,
          Robot::isRed,
          driveSubsystem);
    }
  }

  public static Command getAuto(String autoName) {
    return AutoBuilder.buildAuto(autoName);
  }

  public static SendableChooser<Command> getAutoChooser() {
    return AutoBuilder.buildAutoChooser();
  }

  public static void addEvent(String eventName, Command command) {
    NamedCommands.registerCommand(eventName, CommandUtil.wrappedEventCommand(command));
  }
}
