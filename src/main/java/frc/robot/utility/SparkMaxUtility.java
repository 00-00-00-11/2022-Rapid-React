package frc.robot.utility;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxUtility {

  public static CANSparkMax constructSparkMax(int port, boolean brushless) {
    if (brushless) {
      return new CANSparkMax(port, MotorType.kBrushless);
    } else {
      return new CANSparkMax(port, MotorType.kBrushed);
    }
  }

  public static void runSparkMax(CANSparkMax sparkMax, double speed) {
    sparkMax.set(speed);
  }
}