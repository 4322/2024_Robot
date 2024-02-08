package frc.robot.subsystems.tunnel;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface TunnelInterface extends Subsystem{
    public void feed();

    public void setBrakeMode();

    public void setCoastMode();

    public void stopTunnel();
}
