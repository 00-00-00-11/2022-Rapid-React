package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.LoggingUtil;

public class ColorSubsystem extends SubsystemBase {

    private final I2C.Port i2cPort = I2C.Port.kOnboard; // defining port I2C class reference
    // private final I2C.Port mxpi2cPort = I2C.Port.kMXP; // defining port I2C class reference

    private final ColorSensorV3 m_colorSensorTop = new ColorSensorV3(i2cPort); // TOP SENSOR
    // private final ColorSensorV3 m_colorSensorBottom = new ColorSensorV3(mxpi2cPort); // BOTTOM SENSOR

    // private final ColorMatch m_colorMatcher = new ColorMatch();
    // private final Color kBlueTarget = new Color(0.2275, 0.4736, 0.2991);
    // private final Color kRedTarget = new Color(0.2771, 0.468, 0.256);

    private final NetworkTable table;

    public ColorSubsystem() {
        // m_colorMatcher.addColorMatch(kBlueTarget);
        // m_colorMatcher.addColorMatch(kRedTarget);
        // m_colorMatcher.addColorMatch(Color.kLightCoral);
        table = NetworkTableInstance.getDefault().getTable("Color");
    }

    @Override
    public void periodic() {
        log();
    }

    // public String intakeBallMatcher() { // changed to void from String
    //     // Color detectedColor = m_colorSensor.getColor();
    //     int proximity = m_colorSensor.getProximity();
    //     // SmartDashboard.putString("color ting", "R"+detectedColor.red+" G"+detectedColor.green+" B"+detectedColor.blue);// to get this jit

    //     // 2047 closest ~0.25 inches, 0 farthest ~6 inches
    //     LoggingUtil.logWithNetworkTable(table, "Proximity", proximity);
    //     if (proximity > Constants.ColorConstants.PROXIMITY_THRESHOLD) {
    //         return "blue";
    //     } else {
    //         return "none";
    //     }

    //     ColorMatchResult matchedColor = m_colorMatcher.matchClosestColor(detectedColor);
    //     if (matchedColor.color == kBlueTarget && matchedColor.confidence > .43) {
    //         SmartDashboard.putString("Matched Color", "blue");
    //         SmartDashboard.putNumber("Color Confidence",matchedColor.confidence);
    //         return "blue";
    //     } else if (matchedColor.color == kRedTarget && matchedColor.confidence > .43) {
    //         SmartDashboard.putString("Matched Color", "red");
    //         SmartDashboard.putNumber("Color Confidence",matchedColor.confidence);
    //         return "red";
    //     }
    //     SmartDashboard.putString("Matched Color", "none");
    //     SmartDashboard.putNumber("Color Confidence",matchedColor.confidence);

    // }

    public int getProximityTop() {
        // 2047 closest ~ 0.25 inches; 0 farthest ~ 6 inches
        return m_colorSensorTop.getProximity();
    }

    /*
    public int getProximity(int sensorNumber) {
        // 2047 closest ~ 0.25 inches; 0 farthest ~ 6 inches
        if(sensorNumber==1) {
            return m_colorSensor.getProximity();
        } else if(sensorNumber==2) {
            return m_colorSensor2.getProximity();
        } else {
            return -10; //line should not occur indicates error in PARAMETER CALL
        }

    }

    */

    // public int getProximityBottom() {
    //     return m_colorSensorBottom.getProximity();
    // }

    public void log() {
        LoggingUtil.logWithNetworkTable(table, "Proximity Top", getProximityTop());
        // LoggingUtil.logWithNetworkTable(table, "Proximity Bottom", getProximityBottom());

    }
}
