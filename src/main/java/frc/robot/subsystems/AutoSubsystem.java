// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class AutoSubsystem extends SubsystemBase {
  /** Creates a new AutoSubsystem. */
  public static CANSparkMAX drivebaseMotor;

  public AutoSubsystem() {
    drivebaseMotor = new CANSparkMAX (0, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoder = drivebaseMotor.getEncoder();
    encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void exitTamac () {
    if (encoder.getPosition()*6.0*Math.PI/18.0 < 6) //6.0 ft = wheel diameter, 18.0 ft = gearbox ratio
      drivebaseMotor.set(-0.2);    
  }

  public void stopMotors () {
    drivebaseMotor.set(0);
  }

}
