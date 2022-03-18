// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    if(Robot.isReal()) {
      limelight  = LimelightUtility.constructLimelight(VisionConstants.LIMELIGHT_ANGLE, VisionConstants.LIMELIGHT_HEIGHT, FieldConstants.HIGH_GOAL_HEIGHT, VisionConstants.PIPELINE);
    } else {
      limelight = LimelightUtility.constructLimelightSim(VisionConstants.LIMELIGHT_ANGLE, VisionConstants.LIMELIGHT_HEIGHT, FieldConstants.HIGH_GOAL_HEIGHT, 0, -120, 30);
    }
  }

  public void autoAlignWithGoal(double setpoint) {

    if(Robot.isReal()) {
      limelight.setLEDMode(3);
      LoggingUtil.logWithNetworkTable(table, "LED Status", "FORCE ON");
    }

    double error = (setpoint - limelight.getTx());
    LoggingUtil.logWithNetworkTable(table, "Horizontal Error", error);
    double speed = VisionConstants.ALIGN_KP * error;

    if(error<0) {
      speed = -speed;
    }

    if(speed > VisionConstants.MAX_ALIGN_SPEED) {
      speed = VisionConstants.MAX_ALIGN_SPEED;
    } else if(speed < -VisionConstants.MAX_ALIGN_SPEED) {
      speed = -VisionConstants.MAX_ALIGN_SPEED;
    }

    LoggingUtil.logWithNetworkTable(table, "Alignment Speed", speed);

    if (error > VisionConstants.ALIGN_THRESHOLD) {
      LoggingUtil.logWithNetworkTable(table, "Align Threshold", VisionConstants.ALIGN_THRESHOLD);
      LoggingUtil.logWithNetworkTable(table, "Aligning Status", "ALIGNING");
      RobotContainer.m_driveSubsystem.curveDrive(0.0, speed, true);
    } else if (error < -VisionConstants.ALIGN_THRESHOLD) {
      LoggingUtil.logWithNetworkTable(table, "Align Threshold", VisionConstants.ALIGN_THRESHOLD);
      LoggingUtil.logWithNetworkTable(table, "Aligning Status", "ALIGNING");
      RobotContainer.m_driveSubsystem.curveDrive(0.0, -speed, true);
    } else {
      LoggingUtil.logWithNetworkTable(table, "Aligning Status", "ALIGNED");
      RobotContainer.m_driveSubsystem.curveDrive(0.0, 0.0, false);
      if(Robot.isReal()) {
        limelight.setLEDMode(1);
        LoggingUtil.logWithNetworkTable(table, "LED Status", "FORCE OFF");
      }
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
    LoggingUtil.logWithNetworkTable(table, "Flywheel RPM Setpoint", speeds.getFlywheelRPM());
    LoggingUtil.logWithNetworkTable(table, "Feeder RPM Setpoint", speeds.getFeederRPM());
  }
}
