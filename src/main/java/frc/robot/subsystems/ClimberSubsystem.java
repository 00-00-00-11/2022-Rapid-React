// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  // Initializing motors and limit switches
  CANSparkMax elevatorMotor;
  CANSparkMax anglerMotorLeft;
  CANSparkMax anglerMotorRight;

  DigitalInput elevTopSwitch;
  DigitalInput elevBottomSwitch;

  // Booleans to handle elevator toggle functionality
  boolean elevatorExtended = false;
  boolean elevatorRunning = false;

  // Pulling variables from constants
  double elevSpeed = Constants.ElevatorConstants.ELEVATOR_SPEED;
  double elevMin = Constants.ElevatorConstants.ELEVATOR_MIN;
  double elevMax = Constants.ElevatorConstants.ELEVATOR_MAX;
  double elevMargin = Constants.ElevatorConstants.ELEVATOR_MARGIN;

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
  public void periodic() {
    if(checkLimitSwitch()){

    }
  }

  public void elevatorCalibrate() {

  }

  // Checks if elevator limit switches are triggered (edge of hardware bounds).
  public boolean checkLimitSwitch() {
    return elevBottomSwitch.get() && elevTopSwitch.get();
  }

  public void elevatorExtend() {
    if (elevatorMotor.getEncoder().getPosition() < (elevMax - elevMargin) && elevatorRunning && !checkLimitSwitch()) {
      elevatorMotor.set(elevSpeed);
    } else {
      elevatorMotor.set(0);
      elevatorExtended = true;
      elevatorRunning = false;
    }
  }

  public void elevatorRetract() {
    if (elevatorMotor.getEncoder().getPosition() > (elevMin + elevMargin) && elevatorRunning && !checkLimitSwitch()) {
      elevatorMotor.set(-elevSpeed);
    } else {
      elevatorMotor.set(0);
      elevatorExtended = false;
      elevatorRunning = false;
    }
  }

  public void anglerControl(PS4Controller gamepad) {
    anglerMotorLeft.set(gamepad.getLeftY());
    anglerMotorRight.set(gamepad.getRightY());
  }

  public void climberControl(PS4Controller gamepad) {
    if (gamepad.getL1Button()) {
      elevatorRunning = true;
    } 

    elevatorMotor.set(-0.2*gamepad.getLeftY());
   /* if (elevatorRunning) {
      if (!elevatorExtended) {
        elevatorExtend();
      } else if (elevatorExtended) {
        elevatorRetract();
      } else {
        System.out.println("Something has gone wrong in elevator software bounding");
      }
    }*/

    anglerControl(gamepad);
  }
}

