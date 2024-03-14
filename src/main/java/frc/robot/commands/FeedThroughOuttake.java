package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.noteTracker.NoteTracker;
import frc.robot.subsystems.tunnel.Tunnel;

public class FeedThroughOuttake extends Command {
    private boolean passingTunnel;

    public FeedThroughOuttake() {
        addRequirements(Tunnel.getInstance());
    }

    @Override
    public void execute() {
        Tunnel.getInstance().reverseFeed();

        if (NoteTracker.getInstance().tunnelBeamBroken()) {
            passingTunnel = true;
        }
    }

    @Override
    public boolean isFinished() {
        return passingTunnel && !NoteTracker.getInstance().tunnelBeamBroken();
    }

    @Override
    public void end(boolean interrupted) {
        Tunnel.getInstance().stopTunnel();

    }
}
