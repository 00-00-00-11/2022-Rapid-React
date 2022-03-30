
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ColorSubsystem {
    private final I2C.Port i2cPort = I2C.Port.kOnboard; // defining port I2C class reference
    private final I2C.Port mxpi2cPort = I2C.Port.kMXP; // defining port I2C class reference

    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); // sensor to return color
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = new Color(0.2275, 0.4736, 0.2991);
    private final Color kRedTarget = new Color(0.2771, 0.468, 0.256);

    public ColorSubsystem() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
       // m_colorMatcher.addColorMatch(Color.kLightCoral);
    }

    public String intakeBallMatcher() { // changed to void from String
        Color detectedColor = m_colorSensor.getColor();
        int proximity = m_colorSensor.getProximity();
        SmartDashboard.putString("color ting", "R"+detectedColor.red+" G"+detectedColor.green+" B"+detectedColor.blue);// to get this jit
        SmartDashboard.putString("proxy color", proximity + " idk units"); // gets proximity using IR

        if (proximity < 60) {
            return "blue";
        } else {
            return "none";
        }
        // ColorMatchResult matchedColor = m_colorMatcher.matchClosestColor(detectedColor);
        // if (matchedColor.color == kBlueTarget && matchedColor.confidence > .43) {
        //     SmartDashboard.putString("Matched Color", "blue");
        //     SmartDashboard.putNumber("Color Confidence",matchedColor.confidence);
        //     return "blue";
        // } else if (matchedColor.color == kRedTarget && matchedColor.confidence > .43) {
        //     SmartDashboard.putString("Matched Color", "red");
        //     SmartDashboard.putNumber("Color Confidence",matchedColor.confidence);
        //     return "red"; 
        // }
        // SmartDashboard.putString("Matched Color", "none");
        // SmartDashboard.putNumber("Color Confidence",matchedColor.confidence);

        
    }
}
