// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utility.LimelightUtility;
import frc.robot.utility.LoggingUtil;
import frc.robot.utility.ShooterSpeeds;
import frc.robot.vision.Limelight;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  Limelight limelight;
  NetworkTable table;
  ShooterSpeeds speeds;

  public VisionSubsystem() {
    speeds = new ShooterSpeeds(0.0, 0.0);
    table = NetworkTableInstance.getDefault().getTable("Vision");
    // limelight.setLEDMode(1);

    if (Robot.isReal()) {
      limelight = LimelightUtility.constructLimelight(VisionConstants.LIMELIGHT_ANGLE, VisionConstants.LIMELIGHT_HEIGHT,
          FieldConstants.HIGH_GOAL_HEIGHT, VisionConstants.PIPELINE);
    } else {
      limelight = LimelightUtility.constructLimelightSim(VisionConstants.LIMELIGHT_ANGLE,
          VisionConstants.LIMELIGHT_HEIGHT, FieldConstants.HIGH_GOAL_HEIGHT, 0, -120, 30);
    }
  }

  public void AutoAlignWithGoal(double setpoint) {

    if (Robot.isReal()) {
      limelight.setLEDMode(3);
      LoggingUtil.logWithNetworkTable(table, "LED Status", "FORCE ON");
    }

    double errorx = -(setpoint - limelight.getTx());
    double errory = -(setpoint - limelight.getTy());

    LoggingUtil.logWithNetworkTable(table, "Horizontal Error", errorx);
    LoggingUtil.logWithNetworkTable(table, "Vertical Error", errory);

    double x_adjust = 0;
    if (Math.abs(errorx) > VisionConstants.ALIGN_THRESHOLD) {
      x_adjust = VisionConstants.ALIGN_KP_X * errorx;
    }
    double y_adjust = VisionConstants.ALIGN_KP_Y * errory;

    double leftSpeed = x_adjust + y_adjust;
    double rightSpeed = y_adjust - x_adjust;

    if (x_adjust < .1 || y_adjust < .1) {
      LoggingUtil.logWithNetworkTable(table, "Aligning Status", "FINISHED ALIGNING");
      if (Robot.isReal()) {
        // limelight.setLEDMode(1);
        LoggingUtil.logWithNetworkTable(table, "LED Status", "FORCE OFF");
      }
    } else {
      RobotContainer.m_driveSubsystem.tankDriveAuto(leftSpeed, rightSpeed);
    }


  }

  public boolean isAligned(double setpoint) {
    return Math.abs(setpoint - limelight.getTx()) < VisionConstants.ALIGN_THRESHOLD;
  }

  public ShooterSpeeds calculateShooterSetpoints(double distanceToGoal) {
    return new ShooterSpeeds(0.0, 0.0);
  }

  @Override
  public void periodic() {
    update();
  }

  public void update() {
    LoggingUtil.logWithNetworkTable(table, "tx", limelight.getTx());
    LoggingUtil.logWithNetworkTable(table, "ty", limelight.getTy());
    LoggingUtil.logWithNetworkTable(table, "tv", limelight.getTv());
    LoggingUtil.logWithNetworkTable(table, "Distance", limelight.getDistanceToGoal());
    LoggingUtil.logWithNetworkTable(table, "Flywheel RPM Setpoint", speeds.getFlywheelVelocity());
    LoggingUtil.logWithNetworkTable(table, "Feeder RPM Setpoint", speeds.getFeederVelocity());
  }
}
