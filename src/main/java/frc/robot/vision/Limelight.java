package frc.robot.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;
import java.util.Hashtable;

public class Limelight {

    Hashtable<String, Double> limelightTable = new Hashtable<>();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    HashMap<String, Double> setpoints = new HashMap<>();

    public Limelight(double shooterAngle, double limelightHeight, double goalHeight, double pipeline) {
        limelightTable.put("limelightHeight", limelightHeight);
        limelightTable.put("shooterAngle", shooterAngle);
        limelightTable.put("goalHeightInches", goalHeight);
        limelightTable.put("pipeline", pipeline);

        setpoints.put("Feeder Setpoint", 0.0);
        setpoints.put("Flywheel Setpoint", 0.0);
    }

    public double getDistanceToGoal() {
        return (limelightTable.get("goalHeightInches") - limelightTable.get("limelightHeight") / Math.tan(limelightTable.get("shooterAngle") + limelightTable.get("ty")));
    }

    public double getHorizontalOffset() {
        return limelightTable.get("tx");
    }

    public double getVerticalOffset() {
        return limelightTable.get("ty");
    }

    public void update() {
        limelightTable.put("ty", Units.degreesToRadians(table.getEntry("ty").getDouble(0.0)));
        limelightTable.put("tx", Units.degreesToRadians(table.getEntry("tx").getDouble(0.0)));
        limelightTable.put("tv", Units.degreesToRadians(table.getEntry("tv").getDouble(0.0)));
    }

    public void setLEDMode(int mode) {
        table.getEntry("ledMode").setNumber(mode);
    }

    public void setPipeline(int pipe) {
        table.getEntry("pipeline").setNumber(pipe);
    }

}
