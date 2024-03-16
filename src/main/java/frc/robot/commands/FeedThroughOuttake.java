package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.noteTracker.NoteTracker;
import frc.robot.subsystems.tunnel.Tunnel;

public class FeedThroughOuttake extends Command {
    private boolean passingTunnel;
    private boolean passedTunnel;
    Timer tunnelStopTimer = new Timer();

    public FeedThroughOuttake() {
        addRequirements(Tunnel.getInstance());
    }

    @Override
    public void initialize() {
        passingTunnel = false;
        passedTunnel = false;
    }

    @Override
    public void execute() {
        Tunnel.getInstance().reverseFeed();

        if (NoteTracker.getInstance().tunnelBeamBroken()) {
            passingTunnel = true;
        }

        if (passingTunnel && !NoteTracker.getInstance().tunnelBeamBroken()) {
            Tunnel.getInstance().stopTunnel();
            tunnelStopTimer.start();
            // wait for 0.5 seconds for command to end so that tunnel power
            // won't reverse immediately with default tunnel feed command
            if (tunnelStopTimer.hasElapsed(0.5)) {
                tunnelStopTimer.stop();
                tunnelStopTimer.reset();
                passedTunnel = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return passedTunnel;
    }

    @Override
    public void end(boolean interrupted) {}
}
