// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  RelativeEncoder leftEncoder = leftMaster.getEncoder();
  RelativeEncoder rightEncoder = rightMaster.getEncoder();

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave1);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave1);

  DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
  Field2d field = new Field2d();

  PIDController turnPID =
      new PIDController(
          Constants.DriveConstants.TURN_KP,
          Constants.DriveConstants.TURN_KI,
          Constants.DriveConstants.TURN_KD);

  /** Creates a new Drivebase. */
  public DriveSubsystem() {
    rightMotors.setInverted(true);
    setBrake();
    SmartDashboard.putBoolean("Valet Mode", false);
    turnPID.setSetpoint(0);
    turnPID.setTolerance(5);

    driveTab.add("Differential Drive", m_drive).withWidget(BuiltInWidgets.kDifferentialDrive);
    driveTab.add("Field View", field).withWidget("Field");

    // For each time the motor turns, the robot moves 1 wheel circumference / how
    // many times the motor would need to turn for one rotation, times 60 so the
    // unit is meters per second rather than meters per minute
    leftEncoder.setPositionConversionFactor(
        (60 * Math.PI * Constants.DriveConstants.WHEEL_DIAMETER)
            / Constants.DriveConstants.GEAR_RATIO);
    rightEncoder.setPositionConversionFactor(
        (60 * Math.PI * Constants.DriveConstants.WHEEL_DIAMETER)
            / Constants.DriveConstants.GEAR_RATIO);
  }

  /** Set motors to brake mode */
  public void setBrake() {
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave1.setIdleMode(IdleMode.kBrake);
    // rightSlave2.setIdleMode(IdleMode.kBrake);
    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave1.setIdleMode(IdleMode.kBrake);
    // leftSlave2.setIdleMode(IdleMode.kBrake);
  }

  /** Set motors to coast mode */
  public void setCoast() {
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightSlave1.setIdleMode(IdleMode.kCoast);
    // rightSlave2.setIdleMode(IdleMode.kCoast);
    leftMaster.setIdleMode(IdleMode.kCoast);
    leftSlave1.setIdleMode(IdleMode.kCoast);
    // leftSlave2.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Gets the velocity of the robot by averaging the speeds of the right and left sides
   *
   * @return robot velocity in meters / second
   */
  public double getVelocity() {
    return (rightEncoder.getVelocity() + leftEncoder.getVelocity()) / 2;
  }

  /**
   * Move the robot using tank drive
   *
   * @param left Left side speed (-1..1)
   * @param right Right side speed (-1..1)
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(
        left * Constants.DriveConstants.DRIVE_SPEED, right * Constants.DriveConstants.DRIVE_SPEED);
  }

  /**
   * Curvature drive method for differential drive platform.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param rotation The normalized curvature [-1.0..1.0]. Clockwise is positive.
   * @param turn If set, overrides constant-curvature turning for turn-in-place maneuvers. zRotation
   *     will control turning rate instead of curvature.
   */
  public void curveDrive(double xSpeed, double rotation, boolean turn) {
    m_drive.curvatureDrive(
        xSpeed * Constants.DriveConstants.DRIVE_SPEED,
        rotation * Constants.DriveConstants.TURN_SPEED,
        turn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    field.setRobotPose(odometry.getPoseMeters());
  }

  /** Updates the robot's odometry. */
  private void updateOdometry() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  /**
   * Gets the robot's yaw angle from the gryoscope. The angle is cumulative, meaning may not be
   * between 0 and 360.
   *
   * @return The robot's angle in degrees
   */
  public double getRoboAngle() {
    return gyro.getAngle();
  }

  /**
   * Gets the robot's turn PID controller
   *
   * @return turn PID controller
   */
  public PIDController getTurnPID() {
    return turnPID;
  }

  /**
   * Gets the difference between two angles, wrapped between -180 and 180
   *
   * @param angle1 First angle in degrees
   * @param angle2 Second angle in degrees
   * @return angle between the two specified angles
   */
  public static double getAngleBetween(double angle1, double angle2) {
    double degrees = angle2 - angle1;
    degrees %= 360;
    if (degrees > 180) degrees -= 360;
    if (degrees < -180) degrees += 360;
    return degrees;
  }
}
