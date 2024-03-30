package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;

public class OperatorPresetLED extends Command {
    private Timer outtakePresetTimer = new Timer();
    private LED led;

    public OperatorPresetLED() {
        led = LED.getInstance();
        addRequirements(LED.getInstance());
    }

    @Override
    public void execute() {
        outtakePresetTimer.start();
        led.setLEDState(LED.LEDState.operatorPreset);
    }

    @Override
    public boolean isFinished() {
        return outtakePresetTimer.hasElapsed(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        outtakePresetTimer.stop();
        outtakePresetTimer.reset();
    }
}
