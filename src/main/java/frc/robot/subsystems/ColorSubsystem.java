
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ColorSubsystem {
    private final I2C.Port i2cPort = I2C.Port.kOnboard; // defining port I2C class reference
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); // sensor to return color
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

    public ColorSubsystem() {
        m_colorMatcher.addColorMatch(Color.kBlue);
        m_colorMatcher.addColorMatch(Color.kRed);
       // m_colorMatcher.addColorMatch(Color.kLightCoral);
    }

    public String matchedBall() { // changed to void from String
        Color detectedColor = m_colorSensor.getColor();
        SmartDashboard.putString("color ting", "R"+detectedColor.red+" G"+detectedColor.green+" B"+detectedColor.blue);// to get this jit

        ColorMatchResult matchedColor = m_colorMatcher.matchClosestColor(detectedColor);
        if (matchedColor.color == Color.kBlue && matchedColor.confidence > .43) {
            SmartDashboard.putString("Matched Color", "blue");
            SmartDashboard.putNumber("Color Confidence",matchedColor.confidence);
            return "blue";
        } else if (matchedColor.color == Color.kRed && matchedColor.confidence > .43) {
            SmartDashboard.putString("Matched Color", "red");
            SmartDashboard.putNumber("Color Confidence",matchedColor.confidence);
            return "red"; 
        }
        SmartDashboard.putString("Matched Color", "none");
        SmartDashboard.putNumber("Color Confidence",matchedColor.confidence);
        return "none";

    }
}
