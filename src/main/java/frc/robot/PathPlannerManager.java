package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotChooser.RobotChooser;
import frc.robot.RobotChooser.RobotChooserInterface;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;

public class PathPlannerManager {
  private static PathPlannerManager manager;
  private static RobotChooserInterface robotSpecificConstants =
      RobotChooser.getInstance().getConstants();
  private HashMap<String, Command> autos;

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
                  robotSpecificConstants.getAutoTrajectoryXYkP(),
                  robotSpecificConstants.getAutoTrajectoryXYkI(),
                  robotSpecificConstants.getAutoTrajectoryXYkD(),
                  robotSpecificConstants.getAutoTrajectoryXYkiZ()),
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

  public void addEvent(String eventName, Command command) {
    NamedCommands.registerCommand(eventName, CommandUtil.wrappedEventCommand(command));
  }

  public void preloadAutos() {
    autos = new HashMap<String, Command>();
    for (String autoName : AutoBuilder.getAllAutoNames()) {
      autos.put(autoName, AutoBuilder.buildAuto(autoName));
    }
  }

  public Command getAuto(String autoName) {
    return autos.get(autoName);
  }

  public Pose2d getStartingPoseFromAutoFile(String autoName) {
    return PathPlannerAuto.getStaringPoseFromAutoFile(autoName);
  }

  public Command followChoreoPath(String pathName) {
    return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathName));
  }
}
