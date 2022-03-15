package frc.robot.utility;

import frc.robot.vision.Limelight;

public class LimelightUtility {

    public static Limelight constructLimelight(double shooterAngle, double limelightHeight, double goalHeight, double pipeline) {
        return new Limelight(shooterAngle, limelightHeight, goalHeight, pipeline);
    }
    
}

