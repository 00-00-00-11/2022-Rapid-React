package frc.robot.utility;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.RobotContainer;

public class ShuffleboardDashboard {
    public static void logToDashboard(String tabName, String name, String value) {
        Shuffleboard.getTab(tabName).add(name, value).getEntry();
    }

    public static void logToDashboard(String tabName, String name, double value) {
        Shuffleboard.getTab(tabName).add(name, value).getEntry();
    }

    public static void logToDashboard(String tabName, String name, boolean value) {
        Shuffleboard.getTab(tabName).add(name, value).getEntry();
    }

    public static void log() {

        logToDashboard("Dashboard", "Gyro", RobotContainer.m_driveSubsystem.getHeading());
        logToDashboard("Dashboard", "Intake Extended", RobotContainer.m_intakeSubsystem.isExtended());
        
    }
}
