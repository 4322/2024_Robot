package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeInterface extends Subsystem {
  public void intake();

  public void outtake();

  public void setBrakeMode();

  public void setCoastMode();

  public void stopIntake();
}
