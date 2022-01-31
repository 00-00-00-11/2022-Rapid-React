package frc.robot.utility;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxUtility {

  public static CANSparkMax constructSparkMax(int port, boolean brushless) {
    return new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushed);
  }

  public static void runSparkMax(CANSparkMax sparkMax, double speed) {
    sparkMax.set(speed);
  }
}
