package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggingUtil {
    public static void log(String subsystem, String name, String message) {
        SmartDashboard.putString(subsystem + "/" + name, message);
    }

    public static void log(String subsystem, String name, Double message) {
        SmartDashboard.putNumber(subsystem + "/" + name, message);
    }

    public static void log(String subsystem, String name, Boolean message) {
        SmartDashboard.putBoolean(subsystem + "/" + name, message);
    }

    public static void log(String subsystem, String name, Integer message) {
        SmartDashboard.putNumber(subsystem + "/" + name, message);
    }

    public static void log(String subsystem, String name, Object message) {
        SmartDashboard.putString(subsystem + "/" + name, message.toString());
    }
}
