// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  CANSparkMax elevatorMotor;
  CANSparkMax anglerMotorLeft;
  CANSparkMax anglerMotorRight;

  double ElevSpeed = Constants.ElevatorConstants.ELEVATOR_SPEED;
  
  double AnglerSpeed = Constants.ElevatorConstants.ANGLER_SPEED;

  public ClimberSubsystem() {
    elevatorMotor = new CANSparkMax(
        Constants.RobotMap.CLIMBER_LINEAR_CAN, 
        CANSparkMaxLowLevel.MotorType.kBrushless
      );

    anglerMotorLeft = new CANSparkMax(
        Constants.RobotMap.CLIMBER_LEFT_ARM_CAN, 
        CANSparkMaxLowLevel.MotorType.kBrushless
      );

    anglerMotorRight = new CANSparkMax(
      Constants.RobotMap.CLIMBER_RIGHT_ARM_CAN, 
      CANSparkMaxLowLevel.MotorType.kBrushless
    );

    elevatorMotor.setIdleMode(IdleMode.kBrake);
    anglerMotorLeft.setIdleMode(IdleMode.kBrake);
    anglerMotorRight.setIdleMode(IdleMode.kBrake);

    elevatorMotor.setInverted(false);
    anglerMotorLeft.setInverted(false);
    anglerMotorRight.setInverted(false);
  }

  @Override
  public void periodic() {}

  public void elevatorDriver(PS4Controller joystick) {
    if (-joystick.getLeftY() < 0) {
      elevatorMotor.set(-0.8*joystick.getLeftY());
    } else {
      elevatorMotor.set(-0.8*joystick.getLeftY());
    }
  }
}

