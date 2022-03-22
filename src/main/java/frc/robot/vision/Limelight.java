package frc.robot.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
public class Limelight {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight(double shooterAngle, double limelightHeight, double goalHeight, double pipeline) {
        setPipeline(VisionConstants.PIPELINE);
    }

    public double getDistanceToGoal() {
        return ((VisionConstants.GOAL_HEIGHT - VisionConstants.LIMELIGHT_HEIGHT) / Math.tan(Units.degreesToRadians(VisionConstants.LIMELIGHT_ANGLE + getTy())));
    }

    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public boolean getTv() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public void setLEDMode(int mode) {
        table.getEntry("ledMode").setNumber(mode);
    }

    public void setPipeline(int pipe) {
        table.getEntry("pipeline").setNumber(pipe);
    }

}
