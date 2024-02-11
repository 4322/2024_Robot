package frc.robot.subsystems.NoteChecker;

import edu.wpi.first.wpilibj.DigitalInput;

public class DistanceSensorIOPWM implements DistanceSensorIO {

    private DigitalInput photoelectricSwitch; 
    public DistanceSensorIOPWM(int port){
        photoelectricSwitch = new DigitalInput(port);
    }

    @Override
    public double getDistance() {
        /*
        TODO calculate distance based on pulse length:
        The photoelectricSwitch is a digitalInput that has no direct api in WPILIB
        instead you need to measure the pulse length to derive distance apparently. 
        The pulse length directly correlates to the distance. 

        See: https://www.chiefdelphi.com/t/pwm-input/134154/8
        */
        throw new UnsupportedOperationException("Unimplemented method 'getDistance'");
    }

    @Override
    public void updateInputs(DistanceSensorIOInputs inputs){
        //TODO CalculateDistance based off of pulse width. THERE IS 0 API
    }
    
}
