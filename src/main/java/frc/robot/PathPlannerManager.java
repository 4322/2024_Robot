package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

class PathPlannerManager {

  Drive drive;

  PathConstraints defaultConstraints;
  HolonomicPathFollowerConfig holonomicConfig;
  PPHolonomicDriveController holonomicDriveController;
  AutoBuilder builder;

  HashMap<String, Command> eventMap;
  HashMap<String, Command> autoMap;

  PathPlannerManager(Drive driveSubsystem) {

    drive = driveSubsystem;

    eventMap = new HashMap<String, Command>();

    defaultConstraints =
        new PathConstraints(
            DriveConstants.Auto.autoMaxSpeedMetersPerSecond,
            DriveConstants.Auto.autoMaxAccelerationMetersPerSec2,
            DriveConstants.Auto.autoMaxRotateSpeedMetersPerSecond,
            DriveConstants.Auto.autoMaxAngularAccelerationPerSec2);

    builder = new AutoBuilder();
    holonomicConfig = new HolonomicPathFollowerConfig(
      DriveConstants.Auto.autoMaxSpeedMetersPerSecond, 
      DriveConstants.distWheelMetersR, 
      new ReplanningConfig());

      holonomicDriveController = new PPHolonomicDriveController(            
        new PIDConstants(
            DriveConstants.Trajectory.PIDXY.kP,
            DriveConstants.Trajectory.PIDXY.kI,
            DriveConstants.Trajectory.PIDXY.kD),
        new PIDConstants(
            DriveConstants.Trajectory.PIDR.kP,
            DriveConstants.Trajectory.PIDR.kI,
            DriveConstants.Trajectory.PIDR.kD), 
        DriveConstants.maxSpeedMetersPerSecond,
        DriveConstants.distWheelMetersR);
    
    AutoBuilder.configureHolonomic(
      drive::getPose2d, 
      drive::resetOdometry, 
      drive::getChassisSpeeds,
      drive::setModuleStatesFromChassisSpeeds,
      holonomicConfig,
      Robot::isRed,
      driveSubsystem);
  }

  // OTF = On The Fly
  public FollowPathWithEvents generateCommandOTF(PathConstraints constraints, GoalEndState endState, PathPoint... points) {
    PathPlannerPath path = PathPlannerPath.fromPathPoints(Arrays.asList(points), constraints, endState);
    return generateSingleCommand(path);
  }

  //
  public FollowPathWithEvents generateSingleCommand(String pathName) {
    return generateSingleCommand(pathName, defaultConstraints);
  }

  public FollowPathWithEvents generateSingleCommand(String pathName, PathConstraints constraints) {
    return generateSingleCommand(PathPlannerPath.fromPathFile(pathName));
  }

  //FollowPathWithEvents is decapriated and will be removed. 
  private FollowPathWithEvents generateSingleCommand(
      PathPlannerPath path) {

    /* Will probably be replaced with follow path command
    PPSwerveControllerCommand ppSwerveCommand =
        new PPSwerveControllerCommand(
            trajectory,
            drive::getPose2d,
            drive.getKinematics(),
            new PIDController(
                DriveConstants.Trajectory.PIDXY.kP,
                DriveConstants.Trajectory.PIDXY.kI,
                DriveConstants.Trajectory.PIDXY.kD),
            new PIDController(
                DriveConstants.Trajectory.PIDXY.kP,
                DriveConstants.Trajectory.PIDXY.kI,
                DriveConstants.Trajectory.PIDXY.kD),
            new PIDController(
                DriveConstants.Trajectory.PIDR.kP,
                DriveConstants.Trajectory.PIDR.kI,
                DriveConstants.Trajectory.PIDR.kD),
            drive::setModuleStates,
            drive);
            */

        FollowPathCommand followPathCommand = 
        new FollowPathCommand
        (   
          path,
          drive::getPose2d, 
          drive::getChassisSpeeds,
          drive::setModuleStatesFromChassisSpeeds,
          holonomicDriveController,
          new ReplanningConfig(),
          Robot::isRed, 
          drive
        );

            
    FollowPathWithEvents command = new FollowPathWithEvents(followPathCommand, path, drive::getPose2d);
    return command;
  }



  public Command loadAuto(String pathGroupName, boolean reversed) {
    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup(pathGroupName, reversed, defaultConstraints, defaultConstraints);

    return builder.fullAuto(pathGroup);
  }

  public void addEvent(String eventName, Command command) {
    eventMap.put(eventName, command);
  }
}
