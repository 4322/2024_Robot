package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.drive.Drive;

public class AutoNoteCenterLinePath extends Command {

    private Drive drive;
    private double desiredHeadingAngle;
    private double desiredRobotDirectionY;
    private boolean goingDown; 

    private Integer signOfHeading;

    private boolean outsideBounds;
    public AutoNoteCenterLinePath(boolean goingDown) {
        drive = Drive.getInstance();
        addRequirements(drive);
        this.goingDown = goingDown;

      }

    @Override public void initialize(){
        signOfHeading  = (goingDown ? 1 : -1) * (Robot.isRed() ? 1: -1);
        desiredHeadingAngle = 45* signOfHeading;
        desiredRobotDirectionY = (goingDown ? -1 : 1);
        outsideBounds = false;
    }    

    @Override
    public void execute() {
        double approachSpeed =1; 
        double stoppingDistance = 1;
        Pose2d pose = drive.getPose2d();
        if ((goingDown && pose.getY() < (Constants.fieldWidthMeters - stoppingDistance))
        ||(!goingDown && pose.getY() > (stoppingDistance)))
        {
            drive.driveAutoRotate(
            0,
            approachSpeed * desiredRobotDirectionY,
            desiredHeadingAngle);
        }else{
            outsideBounds = true;
        }
    }

    @Override
    public boolean isFinished() {

        RobotCoordinator coordinator = RobotCoordinator.getInstance();
        return outsideBounds || coordinator.noteInRobot();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
