package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.tunnel.Tunnel;

public class Shoot extends Command{
    private final RobotCoordinator coordinator;
    private final Tunnel tunnel;
    public Shoot()
    {
        coordinator = RobotCoordinator.getInstance();
        tunnel = Tunnel.getInstance();
        addRequirements(tunnel);
    }
    @Override
    public void execute()
    {
        if(coordinator.canShoot() && coordinator.noteInRobot())
        {
            tunnel.feed();
        }
    }
    @Override
    public boolean isFinished() {
        return(!coordinator.noteInRobot()); //once the tunnel beam can no longer 'see' the note, it'll be on the flywheel and can be stopped.
    }
    @Override
    public void end(boolean interrupted) {
        tunnel.stopTunnel();
    }
}
