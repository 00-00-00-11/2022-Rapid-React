package frc.robot.utility;

import frc.robot.simulation.LimelightSim;
import frc.robot.vision.Limelight;

public class LimelightUtility {

    public static Limelight constructLimelight(double shooterAngle, double limelightHeight, double goalHeight, double pipeline) {
        return new Limelight(shooterAngle, limelightHeight, goalHeight, pipeline);
    }

    public static Limelight constructLimelightSim(double shooterAngle, double limelightHeight, double goalHeight, double pipeline, double tx, double ty) {
        return new LimelightSim(shooterAngle, limelightHeight, goalHeight, pipeline, tx, ty);
    }

    public static double getTy (Limelight limelight) {
        return limelight.getVerticalOffset();
    }

    public static double getTx (Limelight limelight) {
        return limelight.getHorizontalOffset();
    }
    
}

