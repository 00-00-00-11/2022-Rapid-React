// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utility.LoggingUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ClimberSubsystem extends SubsystemBase {

    // Initializing motors and limit switches
    *
    CANSparkMax elevatorMotor;*
    CANSparkMax anglerMotorLeft;*
    CANSparkMax anglerMotorRight;**
    DigitalInput limitSwitch;**
    NetworkTable table;** // Booleans to handle elevator toggle functionality
    *
    boolean elevatorExtended = false;** // Pulling variables from constants
    *
    double elevSpeed = Constants.ElevatorConstants.ELEVATOR_SPEED;*

    *
    double anglerSpeed = Constants.ElevatorConstants.ANGLER_SPEED;*

    *

    public ClimberSubsystem() {
     * elevatorMotor = new CANSparkMax(Constants.RobotMap.CLIMBER_LINEAR_CAN,
                
     * CANSparkMaxLowLevel.MotorType.kBrushless);
     * 

        ANSparkMaxLowLevel.MotorType.kBrushless);
     * 
     * anglerMotorRight = new CANSparkMax(Constants.RobotMap.CLIMBER_RIGHT_ARM_CAN,
     * CANSparkMaxLowLevel.MotorType.kBrushless);
     * 
     * limitSwitch = new DigitalInput(0);
     * 
     * table = NetworkTableInstance.getDefault().getTable("Elevator");
     * 
     * elevatorMotor.setIdleMode(IdleMode.kBrake);
     * anglerMotorLeft.setIdleMode(IdleMode.kBrake); 
     * anglerMotorRight.setIdleMode(IdleMode.kBrake);
     * 
     * elevatorMotor.setInverted(false);
     * anglerMotorLeft.setInverted(false);
     * anglerMotorRight.setInverted(false);
     * 
     * elevatorMotor.getEncoder().setPosition(0); // to avoid troubleshootin issues
     * }**

    @Override
     * public void periodic() {
     * if (limitSwitchIsTriggered()) {
     * elevatorMotor.getEncoder().setPosition(0);
     * 
     * }
     * 
     * log();
     * }**

    /**
     * Checks if elevator limit switches are triggered (edge of hardware bounds)
     * 
     * @return Limit switch is triggered
     */
    public boolean limitSwitchIsTriggered() {
        return !limitSwitch.get();
    }

    public boolean retractElevator() {
        if (limitSwitchIsTriggered()) {
            elevatorMotor.set(0);
     

        } else {
            elevatorMotor.set(-1);
       
     *      LoggingUtil.logWithNetworkTable(table, "Homing", "Doing");
            return false;
        }
    }

    public TrapezoidProfileCommand getTrapezoidCommand() {
        TrapezoidProfile trap = new TrapezoidProfile(TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION), new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(0, 0));
    }

    /**
     * Controls climber
     * 
     * @param gamepad Operator gamepad
     */

    /*
     * public void climberControl(PS4Controller gamepad) {
     * double elevatorInput = gamepad.getR2Axis() - gamepad.getL2Axis();
     * double leftArmInput = gamepad.getLeftY();
     * double rightArmInput = gamepad.getRightY();
     * double position = elevatorMotor.getEncoder().getPosition();
     * if (position <= 0) {
     * if (elevatorInput > 0) {
     * LoggingUtil.logWithNetworkTable(table, "State", "Moving, Up");
     * elevatorMotor.set(elevatorInput);
     * } else {
     * LoggingUtil.logWithNetworkTab le(table, "State", "Not Moving, At Bottom");
     * elevatorMotor.set(0);
     * }
     * } else if (position >= Constants.ElevatorConstants.ELEVATOR_MAX) {
     * if (elevatorInput < 0) {
     * LoggingUtil.logWithNetworkTable(table, "State", "Moving, Down");
     * elevatorMotor.set(elevatorInput);
     * } else {
     * LoggingUtil.logWithNetworkTable(table, "State", "Not Moving, At Top");
     * elevatorMotor.set(0);
     * }
     * } else {
     * elevatorMotor.set(elevatorInput);
     * LoggingUtil.logWithNetworkTable(table, "State", "Moving");
     * }
     * 
     * anglerMotorLeft.set(leftArmInput*.1);
     * anglerMotorRight.set(rightArmInput*.1*-1);
     * 
     * LoggingUtil.logWithNetworkTable(table, "Input", elevatorInput);
     * LoggingUtil.logWithNetworkTable(table, "L Arm Pos",
     * anglerMotorLeft.getEncoder().getPosition());
     * LoggingUtil.logWithNetworkTable(table, "R Arm Pos",
     * anglerMotorRight.getEncoder().getPosition());
     * 
     * }
     * 
     * public void log() {
     * LoggingUtil.logWithNetworkTable(table, "Input", 0d);
     * LoggingUtil.logWithNetworkTable(table, "State", "Disabled");
     * LoggingUtil.logWithNetworkTable(table, "Position",
     * elevatorMotor.getEncoder().getPosition());
     * LoggingUtil.logWithNetworkTable(table, "Limit Switch",
     * limitSwitchIsTriggered());
     * }
     * 
     */
}
