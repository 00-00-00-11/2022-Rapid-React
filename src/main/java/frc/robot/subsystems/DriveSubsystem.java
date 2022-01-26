// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;

//PID code from 2021 repo that are needed for auto code
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class DriveSubsystem extends SubsystemBase {
  AHRS gyro = new AHRS();
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

  RelativeEncoder leftMasterEncoder = leftMaster.getEncoder();

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave1);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave1);

  DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  PIDController turnPID =
      new PIDController(
          Constants.DriveConstants.turnKP,
          Constants.DriveConstants.turnKI,
          Constants.DriveConstants.turnKD);

  //PID code from 2021 repo that are needed for auto code
  final double gearRatio = 10.81;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
	DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

	SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.145, 2.8, 0.425);

	PIDController leftPIDController = new PIDController(2.32, 0, 0);
	PIDController rightPIDController = new PIDController(2.32, 0, 0);

	Pose2d pose;

  edu.wpi.first.wpilibj.smartdashboard.Field2d m_field = new edu.wpi.first.wpilibj.smartdashboard.Field2d();

  /** Creates a new Drivebase. */
  public DriveSubsystem() {
    rightMotors.setInverted(true);
    setBrake();
    SmartDashboard.putBoolean("Valet Mode", false);
    turnPID.setSetpoint(0);
    turnPID.setTolerance(1);

    leftMasterEncoder.setPosition(0.0);

    //PID code from 2021 repo that are needed for auto code
    SmartDashboard.putData("Field", m_field);
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
    m_drive.curvatureDrive(xSpeed, rotation, turn);
  }

  @Override
  public void periodic() {
    //PID code from 2021 repo that are needed for auto code
    pose = odometry.update(getHeading(),
				leftMaster.getEncoder().getPosition() / gearRatio * 2 * Math.PI * Units.inchesToMeters(3.0),
				rightMaster.getEncoder().getPosition() / gearRatio * 2 * Math.PI * Units.inchesToMeters(3.0));
		m_field.setRobotPose(odometry.getPoseMeters());

		SmartDashboard.putNumber("odometry x", odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("odometry y", odometry.getPoseMeters().getY());
		SmartDashboard.putNumber("heading", -gyro.getAngle());
		SmartDashboard.putString("wheel speeds", getSpeeds().toString());
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

  public double getEncoderPosition () {
    return leftMasterEncoder.getPosition();
  }

  //PID code from 2021 repo that are needed for auto code
  public Pose2d getPose() {
		return pose;
	}

  public void resetEncoders() {
		leftMaster.getEncoder().setPosition(0.0);
		rightMaster.getEncoder().setPosition(0.0);
	}
  
  public void resetOdometry(Pose2d pose) {
		resetEncoders();
		odometry.resetPosition(pose, gyro.getRotation2d());
	}

  public DifferentialDriveWheelSpeeds getSpeeds() {
		return new DifferentialDriveWheelSpeeds(
				leftMaster.getEncoder().getVelocity() / gearRatio * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
				rightMaster.getEncoder().getVelocity() / gearRatio * 2 * Math.PI * Units.inchesToMeters(3.0) / 60);
	}

	public DifferentialDriveKinematics getKinematics() {
		return kinematics;
	}

	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(-gyro.getYaw());
	}

	public SimpleMotorFeedforward getFeedforward() {
		return feedForward;
	}

	public PIDController getLeftPIDController() {
		return leftPIDController;
	}

	public PIDController getRightPIDController() {
		return rightPIDController;  
  }

  public void setOutput(double leftVolts, double rightVolts) {
		leftMaster.set(leftVolts / 12);
		rightMaster.set(rightVolts / 12);
	}
}
