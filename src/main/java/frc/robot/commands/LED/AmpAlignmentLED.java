package frc.robot.commands.LED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveManual.DriveManual;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualState;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.limelight.Limelight;

public class AmpAlignmentLED extends Command {
    private DriveManual driveManual;

    public AmpAlignmentLED(DriveManual driveManual) {
        this.driveManual = driveManual;
        addRequirements(LED.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        final int tagID;
        if (Robot.isRed()) {
            tagID = Constants.FieldConstants.redAmpTagID;
        }
        else {
            tagID = Constants.FieldConstants.blueAmpTagID;
        }

        if (Limelight.getOuttakeInstance().getSpecifiedTagVisible(tagID)) {
            if (Math.abs(Limelight.getOuttakeInstance().getTag(tagID).tx) <= 5.0 ) {
                LED.getInstance().setLEDState(LED.LEDState.alignedWithAmp);
            }
            else {
                LED.getInstance().setLEDState(LED.LEDState.aligningWithAmp);
            }
        }
        else {
            LED.getInstance().setLEDState(LED.LEDState.idle);
        }
    }

    @Override
    public boolean isFinished() {
        return driveManual.getState() != DriveManualState.AMP;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
