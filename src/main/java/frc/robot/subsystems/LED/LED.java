package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
    public LedIO io;

    public LED() {
        switch (Constants.currentMode) {
            case REAL:
                io = new LedIOReal();
                break;

            case SIM:
                break;
            
            case REPLAY:
                break;
        }

        if (io == null) {
            io = new LedIO() {};
        }
    }

    @Override
    public void periodic() {
        io.rainbowAnimate(1, 0.3, 5, 0);
        io.flashAnimate(255, 0, 0, 0.6, 5, 6);
        
    }
}
