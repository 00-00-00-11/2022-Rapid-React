// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ClimberSubsystem extends SubsystemBase {

  // Initializing motors and limit switches
  CANSparkMax elevatorMotor;
  CANSparkMax anglerMotorLeft;
  CANSparkMax anglerMotorRight;

  DigitalInput limitSwitch;

  // Booleans to handle elevator toggle functionality
  boolean elevatorExtended = false;

  // Pulling variables from constants
  double elevSpeed = Constants.ElevatorConstants.ELEVATOR_SPEED;

  double AnglerSpeed = Constants.ElevatorConstants.ANGLER_SPEED;

  public ClimberSubsystem() {
    elevatorMotor = new CANSparkMax(
        Constants.RobotMap.CLIMBER_LINEAR_CAN,
        CANSparkMaxLowLevel.MotorType.kBrushless);

    anglerMotorLeft = new CANSparkMax(
        Constants.RobotMap.CLIMBER_LEFT_ARM_CAN,
        CANSparkMaxLowLevel.MotorType.kBrushless);

    anglerMotorRight = new CANSparkMax(
        Constants.RobotMap.CLIMBER_RIGHT_ARM_CAN,
        CANSparkMaxLowLevel.MotorType.kBrushless);

    limitSwitch = new DigitalInput(0);

   //elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ElevatorConstants.ELEVATOR_MAX); // FIXME check direction and elevator max
    //elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,0);// FIXME check direction and elevator max

    elevatorMotor.setIdleMode(IdleMode.kBrake);
    anglerMotorLeft.setIdleMode(IdleMode.kBrake);
    anglerMotorRight.setIdleMode(IdleMode.kBrake);

    elevatorMotor.setInverted(false);
    anglerMotorLeft.setInverted(false);
    anglerMotorRight.setInverted(false);

    elevatorMotor.getEncoder().setPosition(0); //to avoid troubleshootin issues
  }

  @Override
  public void periodic() {
    if(limitSwitchIsTriggered()){
      elevatorMotor.getEncoder().setPosition(0);
    }
    SmartDashboard.putNumber("ELEVATOR POSITION",elevatorMotor.getEncoder().getPosition());
  }

  public void elevatorCalibrate() {

  }

  // Checks if elevator limit switches are triggered (edge of hardware bounds).
  public boolean limitSwitchIsTriggered() {
    return !limitSwitch.get();
  }

  public void elevatorExtend() {
    if (elevatorMotor.getEncoder().getPosition() < ElevatorConstants.ELEVATOR_MAX) {
      elevatorMotor.set(elevSpeed);
    } else {
      elevatorMotor.set(0);
      elevatorExtended = true;
    }
  }

  public void elevatorRetract() {
    if (!limitSwitchIsTriggered()) {
      elevatorMotor.set(-elevSpeed);
    } else {
      elevatorMotor.set(0);
      elevatorExtended = false;
    }
  }

  public void anglerControl(PS4Controller gamepad) {
    anglerMotorLeft.set(gamepad.getLeftY());
    anglerMotorRight.set(gamepad.getRightY());
  }
  
  public void reachedLowLimit(){
    
  }

  public void climberControl(PS4Controller gamepad) {
    double input = -gamepad.getLeftY();
    double position = elevatorMotor.getEncoder().getPosition();
    if(position <= 0){
      if(input>0){
        SmartDashboard.putString("ELEVATOR STATE", "MOVING UP FROM BOTTOM");
        elevatorMotor.set(input);
      }else {
        SmartDashboard.putString("ELEVATOR STATE", "AT BOTTOM, CAN NOT MOVE FURTHER");
        elevatorMotor.set(0);

      }
    }else if(position >= Constants.ElevatorConstants.ELEVATOR_MAX) {
      if(input < 0){
        SmartDashboard.putString("ELEVATOR STATE", "MOVING DOWN FROM TOP");
        elevatorMotor.set(input);
      }else {
        SmartDashboard.putString("ELEVATOR STATE", "AT TOP, CAN NOT MOVE FURTHER");
        elevatorMotor.set(0);
      }
    }else{
      elevatorMotor.set(input);
      SmartDashboard.putString("ELEVATOR STATE", "MOVING");

    }
    SmartDashboard.putNumber("ELEVATOR INPUT",input);


   /* if (elevatorRunning) {
      if (!elevatorExtended) {
        elevatorExtend();
      } else if (elevatorExtended) {
        elevatorRetract();
      } else {
        System.out.println("error in elevator marginal bounding");  //error if elevator not extended or retracted
      }
    }*/

  }
}