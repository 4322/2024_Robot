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
import frc.robot.Constants.DriveConstants.Trajectory.PIDR;
import frc.robot.Constants.DriveConstants.Trajectory.PIDXY;
import frc.robot.subsystems.drive.Drive;

class PathPlannerManager {

  public static void init(Drive driveSubsystem) {
    if (!AutoBuilder.isConfigured()) {
      HolonomicPathFollowerConfig holonomicConfig = new HolonomicPathFollowerConfig(
          new PIDConstants(PIDXY.kP, PIDXY.kI, PIDXY.kD, PIDXY.iZ),
          new PIDConstants(PIDR.kP, PIDR.kI, PIDR.kD, PIDR.iZ), DriveConstants.Auto.autoMaxSpeedMetersPerSecond,
          DriveConstants.distWheelMetersR, new ReplanningConfig());

      AutoBuilder.configureHolonomic(driveSubsystem::getPose2d, driveSubsystem::resetOdometry,
          driveSubsystem::getChassisSpeeds, driveSubsystem::setModuleStatesFromChassisSpeeds, holonomicConfig,
          Robot::isRed, driveSubsystem);
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
