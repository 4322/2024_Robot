package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.tunnel.Tunnel;

public class EjectThroughOuttake extends Command {
    Outtake outtake;
    Tunnel tunnel;

    public EjectThroughOuttake() {
        outtake = Outtake.getInstance();
        tunnel = Tunnel.getInstance();
        
        addRequirements(outtake, tunnel);
    }

    @Override
    public void initialize() {
        outtake.pivot(Constants.OuttakeConstants.reverseSoftLimitThresholdRotations);
        outtake.outtake(Constants.OuttakeConstants.ejectOuttakeRPS);
    }

    @Override
    public void execute() {
        if (RobotCoordinator.getInstance().pivotAtPosition()) {
            tunnel.feed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stopOuttake();
        outtake.stopPivot();
        tunnel.stopTunnel();
        
    }


}
