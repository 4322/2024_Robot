package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.LED.LED;

public class LEDState extends Command {
    private LED led;
    private Timer aprilTagLossTimer = new Timer();
    public LEDState() {
        led = LED.getInstance();
        addRequirements(LED.getInstance());
    }

    @Override
    public void initialize() {
        aprilTagLossTimer.restart();
    }

    @Override
    public void execute() {
        // LEDs while driving
        if (DriverStation.isEnabled()) {
            if (RobotCoordinator.getInstance().canSmartShoot()
                && RobotCoordinator.getInstance().noteInFiringPosition()) {
                aprilTagLossTimer.reset();
                led.setLEDState(LED.LEDState.noteReadyToShoot);
            }
            // If outtake is at right pos and speed, wait for short amount of 
            // time to switch LED state from ready to shoot in order to stop flickering LEDs.
            else if (RobotCoordinator.getInstance().canShoot() 
                && led.getLEDState() == LED.LEDState.noteReadyToShoot 
                    && !aprilTagLossTimer.hasElapsed(Constants.LimelightConstants.aprilTagLossThresholdSec)) {
                led.setLEDState(LED.LEDState.noteReadyToShoot);
            }
            else if (RobotCoordinator.getInstance().canShoot() 
                && RobotCoordinator.getInstance().noteInFiringPosition()) {
                led.setLEDState(LED.LEDState.outtakeAtFiringPosition);
            }
            else if (RobotCoordinator.getInstance().noteInRobot()) {
                led.setLEDState(LED.LEDState.noteInRobot);
            } else {
                led.setLEDState(LED.LEDState.idle);
            }
        } 
        // LEDs while disabled
        else {
            if (!RobotCoordinator.getInstance().isInitialized()) {
                led.setLEDState(LED.LEDState.notInitialized);
            } else if (RobotCoordinator.getInstance().deployInCoast()
            && RobotCoordinator.getInstance().pivotInCoast()) {
                led.setLEDState(LED.LEDState.coastMode);
            } else if (!RobotCoordinator.getInstance().deployInCoast()
            && !RobotCoordinator.getInstance().pivotInCoast()) {
                led.setLEDState(LED.LEDState.brakeMode);
            } else {
                led.setLEDState(LED.LEDState.idle);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
