package frc.robot.subsystems.NoteChecker;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteChecker extends SubsystemBase {

    private static NoteChecker noteCheckerSubsystem;

    private DistanceSensorIO intakeDistanceSensor; 
    private DistanceSensorIO tunnelDistanceSensor;


    public static NoteChecker getInstance(){
        if(noteCheckerSubsystem == null){
            noteCheckerSubsystem = new NoteChecker();
        }

        return noteCheckerSubsystem;
    }
    
    private NoteChecker(){
        switch (Constants.currentMode) {
            case REAL:
                //TODO assign specific ports within constants currently just 0 and 1. 
                intakeDistanceSensor = new DistanceSensorIOPWM(Constants.NoteCheckerConstants.intakeDistanceSensorPort);
                tunnelDistanceSensor = new DistanceSensorIOPWM(Constants.NoteCheckerConstants.tunnelDistanceSensorPort);
            break;
            case SIM:
            break;
            case REPLAY:
            break;
        }
    }

    public boolean hasNote(){
        throw new UnsupportedOperationException("");
    }

    public boolean isInPosition(){
        throw new UnsupportedOperationException("");
    }
}
