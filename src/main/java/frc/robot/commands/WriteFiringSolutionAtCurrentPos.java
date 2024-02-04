package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtakePivot.OuttakePivot;
import frc.robot.subsystems.outtakePivot.OuttakePivotIO;
import frc.utility.PositionVector;

public class WriteFiringSolutionAtCurrentPos extends InstantCommand {
    
    private Outtake outtake;
    private Drive drive;
    private PositionVector vector; 
    private FiringSolutionManager firingSolutionManager;
    private OuttakePivot outtakePivot;
    double shotAngle;
    double shotMag;

    public WriteFiringSolutionAtCurrentPos(Outtake outtakeSubsystem, Drive drivesubsystem, OuttakePivot pivot, PositionVector vectorVal){
        outtake = outtakeSubsystem;
        drive = drivesubsystem;
        outtakePivot = pivot;
        vector = vectorVal;
    }

    
    @Override
    public void initialize() {
        Translation2d rawTranslation = vector.getVectorToSpeaker(drive.getPose2d().getX(), drive.getPose2d().getY());
        shotAngle = rawTranslation.getAngle().getDegrees();
        //Calculates magnitude from x and y vals
        shotMag = Math.sqrt(rawTranslation.getX()*rawTranslation.getX() + rawTranslation.getY() * rawTranslation.getY());
        FiringSolution solution = new FiringSolution(shotMag, shotAngle, outtake.getTargetRPM(), outtakePivot.getTarget());
        firingSolutionManager.addSolution(solution);
        firingSolutionManager.addSolution(solution);
    }
  
    @Override
    public boolean isFinished() {
      return true;
    }
  
    @Override
    public void end(boolean interrupted) {}
    
}
