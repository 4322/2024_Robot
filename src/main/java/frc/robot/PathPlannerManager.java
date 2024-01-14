package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

class PathPlannerManager {

  Drive drive;

  PathConstraints defaultConstraints;
  SwerveAutoBuilder builder;

  HashMap<String, Command> eventMap;
  HashMap<String, Command> autoMap;

  PathPlannerManager(Drive driveSubsystem) {

    drive = driveSubsystem;

    eventMap = new HashMap<String, Command>();

    defaultConstraints = new PathConstraints(DriveConstants.Auto.autoMaxSpeedMetersPerSecond,
        DriveConstants.Auto.autoMaxAccelerationMetersPerSec2);

    builder = new SwerveAutoBuilder(drive::getPose2d, drive::resetOdometry, drive.getKinematics(),
        new PIDConstants(DriveConstants.Trajectory.PIDXY.kP, DriveConstants.Trajectory.PIDXY.kI,
            DriveConstants.Trajectory.PIDXY.kD),
        new PIDConstants(DriveConstants.Trajectory.PIDR.kP, DriveConstants.Trajectory.PIDR.kI,
            DriveConstants.Trajectory.PIDR.kD),
        drive::setModuleStates, eventMap, true, drive);

  }

  // OTF = On The Fly
  public FollowPathWithEvents generateCommandOTF(PathConstraints constraints, PathPoint... points) {
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(constraints, Arrays.asList(points));
    return generateSingleCommand(trajectory, false);
  }

  public FollowPathWithEvents generateSingleCommand(String pathName, boolean reversed) {
    return generateSingleCommand(pathName, reversed, defaultConstraints);
  }

  public FollowPathWithEvents generateSingleCommand(String pathName, boolean reversed,
      PathConstraints constraints) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, constraints);
    return generateSingleCommand(trajectory, reversed);
  }

  private FollowPathWithEvents generateSingleCommand(PathPlannerTrajectory trajectory,
      boolean reversed) {

    PPSwerveControllerCommand ppSwerveCommand =
        new PPSwerveControllerCommand(trajectory, drive::getPose2d, drive.getKinematics(),
            new PIDController(DriveConstants.Trajectory.PIDXY.kP,
                DriveConstants.Trajectory.PIDXY.kI, DriveConstants.Trajectory.PIDXY.kD),
            new PIDController(DriveConstants.Trajectory.PIDXY.kP,
                DriveConstants.Trajectory.PIDXY.kI, DriveConstants.Trajectory.PIDXY.kD),
            new PIDController(DriveConstants.Trajectory.PIDR.kP, DriveConstants.Trajectory.PIDR.kI,
                DriveConstants.Trajectory.PIDR.kD),
            drive::setModuleStates, drive);

    FollowPathWithEvents command =
        new FollowPathWithEvents(ppSwerveCommand, trajectory.getMarkers(), eventMap);
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