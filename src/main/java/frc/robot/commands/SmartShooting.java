package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.outtake.Outtake;

public class SmartShooting extends Command {
    private final Outtake outtake;
    private final Limelight outtakeLimelight;
    private final RobotCoordinator coordinator;

    public SmartShooting() {
        outtake = Outtake.getInstance();
        outtakeLimelight = Limelight.getOuttakeInstance();
        coordinator = RobotCoordinator.getInstance();
        addRequirements(outtake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        final FiringSolution solution;
        final Pose2d botPoseToSpeaker;

        if (Robot.isRed()) {
            botPoseToSpeaker = outtakeLimelight.getTargetPose3DToBot(Constants.FieldConstants.redSpeakerCenterTagID).toPose2d();
        }
        else {
            botPoseToSpeaker = outtakeLimelight.getTargetPose3DToBot(Constants.FieldConstants.blueSpeakerCenterTagID).toPose2d();
        }
        
        double magToSpeaker = botPoseToSpeaker.getTranslation().getNorm();
        double degreesToSpeaker = botPoseToSpeaker.getRotation().getDegrees();
        solution = FiringSolutionManager.getInstance().calcSolution(magToSpeaker, degreesToSpeaker);

        if (coordinator.canPivot()) {
            outtake.pivot(solution.getShotRotations(), true);
        }

        if (coordinator.canShoot()) {
            outtake.outtake(solution.getFlywheelSpeed());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
