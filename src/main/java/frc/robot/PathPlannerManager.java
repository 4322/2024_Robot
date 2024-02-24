package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotChooser.RobotChooser;
import frc.robot.RobotChooser.RobotChooserInterface;
import frc.robot.subsystems.drive.Drive;

public class PathPlannerManager {
  private static RobotChooserInterface robotSpecificConstants =
      RobotChooser.getInstance().getConstants();

  private static PathPlannerManager manager;

  public static PathPlannerManager getInstance() {
    if (manager == null) {
      manager = new PathPlannerManager();
    }
    return manager;
  }

  private PathPlannerManager() {
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
          Drive.getInstance()::getPose2d,
          Drive.getInstance()::resetOdometry,
          Drive.getInstance()::getChassisSpeeds,
          Drive.getInstance()::setModuleStatesFromChassisSpeeds,
          holonomicConfig,
          Robot::isRed,
          Drive.getInstance());
    }
  }

  public Command getAuto(String autoName) {
    return AutoBuilder.buildAuto(autoName);
  }

  public SendableChooser<Command> getAutoChooser() {
    return AutoBuilder.buildAutoChooser();
  }

  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  public void addEvent(String eventName, Command command) {
    NamedCommands.registerCommand(eventName, CommandUtil.wrappedEventCommand(command));
  }
}
