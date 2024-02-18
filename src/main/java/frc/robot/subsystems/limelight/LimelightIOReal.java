package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.utility.OrangeMath;

public class LimelightIOReal implements LimelightIO{
    private Limelight intakeLimelight;
    private Limelight outtakeLimelight;

    public LimelightIOReal() {
        if (Constants.limelightEnabled) {
            intakeLimelight = new Limelight();
            outtakeLimelight = new Limelight();
        }
    }
}
