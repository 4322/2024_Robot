package frc.robot.subsystems.climber;

import frc.robot.Constants;

public class Climber {
    private static Climber climber;
    private ClimberIO io;
    public static Climber getInstance()
    {
        if(climber == null)
        {
            climber = new Climber();
        }
        return climber;
    }

    private Climber()
    {
        switch(Constants.currentMode)
        {
            case REAL:
                if(Constants.climberEnabled){
                    io = new ClimberIOReal();
                }
                break;
            case SIM:
                break;
            case REPLAY:
                break;
        }
        if(io == null)
        {
            io = new ClimberIO() {};
        }
    }
}
