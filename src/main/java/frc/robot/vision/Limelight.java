package frc.robot.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Hashtable;

public class Limelight {

    Hashtable<String, Double> limelightTable = new Hashtable<>();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight(double shooterAngle, double limelightHeight, double goalHeight, double pipeline) {
        limelightTable.put("limelightHeight", limelightHeight);
        limelightTable.put("shooterAngle", shooterAngle);
        limelightTable.put("goalHeightInches", goalHeight);
        limelightTable.put("pipeline", pipeline);    
    }

    public double distanceToGoal() {
        return (limelightTable.get("goalHeightInches") - limelightTable.get("limelightHeight") / Math.tan(limelightTable.get("shooterAngle") + limelightTable.get("ty")));
    }

    public void updateTy() {
        limelightTable.put("ty", Units.degreesToRadians(table.getEntry("ty").getDouble(0.0)));
    }

    public void setLEDMode(int mode) {
        table.getEntry("ledMode").setNumber(mode);
    }

    public void setPipeline(int pipe) {
        table.getEntry("pipeline").setNumber(pipe);
    }
}
