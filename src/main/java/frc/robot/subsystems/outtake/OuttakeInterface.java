package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface OuttakeInterface extends Subsystem {
  public void outtake(double targetRPM);

  public void stopOuttake();

  public void setCoastMode();

  public void setBrakeMode();

  public boolean isFlyWheelUpToSpeed();

  public double getTargetRPM();
}
