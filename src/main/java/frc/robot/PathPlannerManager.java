package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotChooser.RobotChooser;
import frc.robot.RobotChooser.RobotChooserInterface;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;

public class PathPlannerManager {
  private static RobotChooserInterface robotSpecificConstants =
      RobotChooser.getInstance().getConstants();

  private HashMap<String, Command> autos = new HashMap<>();

  public PathPlannerManager(Drive driveSubsystem) {
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

  public void loadAutos() {
    // for (String autoName : AutoBuilder.getAllAutoNames()) {
    //   autos.put(autoName, AutoBuilder.buildAuto(autoName));
    // }
  }

  public Command getAuto(String autoName) {
    return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(autoName));
  }

  public void addEvent(String eventName, Command command) {
    NamedCommands.registerCommand(eventName, CommandUtil.wrappedEventCommand(command));
  }
}
