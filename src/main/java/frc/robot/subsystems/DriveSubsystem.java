// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  CANSparkMax leftMaster =
      new CANSparkMax(
          Constants.DriveConstants.LEFT_MASTER_CAN, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightMaster =
      new CANSparkMax(
          Constants.DriveConstants.RIGHT_MASTER_CAN, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftSlave1 =
      new CANSparkMax(
          Constants.DriveConstants.LEFT_SLAVE_CAN1, CANSparkMaxLowLevel.MotorType.kBrushless);
  // CANSparkMax leftSlave2 = new
  // CANSparkMax(Constants.DriveConstants.LEFT_SLAVE_CAN2,
  // CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave1 =
      new CANSparkMax(
          Constants.DriveConstants.RIGHT_SLAVE_CAN1, CANSparkMaxLowLevel.MotorType.kBrushless);
  // CANSparkMax rightSlave2 = new
  // CANSparkMax(Constants.DriveConstants.RIGHT_SLAVE_CAN2,
  // CANSparkMaxLowLevel.MotorType.kBrushless);

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave1);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave1);

  DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

  PIDController turnPID =
      new PIDController(
          Constants.DriveConstants.turnKP,
          Constants.DriveConstants.turnKI,
          Constants.DriveConstants.turnKD);

  /** Creates a new Drivebase. */
  public DriveSubsystem() {
    rightMotors.setInverted(true);
    setBrake();
    SmartDashboard.putBoolean("Valet Mode", false);
    turnPID.setSetpoint(0);
    turnPID.setTolerance(5);

  }

  public void setBrake() {
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave1.setIdleMode(IdleMode.kBrake);
    // rightSlave2.setIdleMode(IdleMode.kBrake);
    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave1.setIdleMode(IdleMode.kBrake);
    // leftSlave2.setIdleMode(IdleMode.kBrake);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public void curveDrive(double xSpeed, double rotation, boolean turn) {
    m_drive.curvatureDrive(
        xSpeed * Constants.DriveConstants.DRIVE_SPEED,
        rotation * Constants.DriveConstants.TURN_SPEED,
        turn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveTab.add("Differential Drive", m_drive).withWidget(BuiltInWidgets.kDifferentialDrive);
  }

  public double getRoboAngle() {
    return gyro.getAngle();
  }

  public double getAngleBetween(double current, double target) {
    double degrees = target - current;
    degrees %= 360;
    if (degrees > 180) degrees -= 360;
    if (degrees < -180) degrees += 360;
    return degrees;
  }

  public PIDController getTurnPID() {
    return turnPID;
  }
}
