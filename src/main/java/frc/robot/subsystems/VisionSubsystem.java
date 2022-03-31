// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  public void log(){
    double setpoint = 0;
    double x_error = -(setpoint - limelight.getTx());
    double y_error = -(setpoint - limelight.getTy());

    LoggingUtil.logWithNetworkTable(table, "Auto Align", false);
    LoggingUtil.logWithNetworkTable(table, "LED Status", "Off");

    LoggingUtil.logWithNetworkTable(table, "X Err", x_error);
    LoggingUtil.logWithNetworkTable(table, "Y Err", y_error);

    double x_adjust = 0;
    if (Math.abs(x_error) > VisionConstants.ALIGN_THRESHOLD) {
      x_adjust = VisionConstants.ALIGN_KP_X * x_error;
    }
    double y_adjust = VisionConstants.ALIGN_KP_Y * y_error;

    LoggingUtil.logWithNetworkTable(table, "X Adjust", x_adjust);
    LoggingUtil.logWithNetworkTable(table, "Y Adjust", y_adjust);

    double leftSpeed = x_adjust + y_adjust;
    double rightSpeed = y_adjust - x_adjust;

    LoggingUtil.logWithNetworkTable(table, "Auto State", "Aligning");
    LoggingUtil.logWithNetworkTable(table, "Left Speed", leftSpeed);
    LoggingUtil.logWithNetworkTable(table, "Right Speed", rightSpeed);

  }

  public void autoAlignWithGoal(double setpoint) {
    LoggingUtil.logWithNetworkTable(table, "Auto Align", true);

    double x_error = -(setpoint - limelight.getTx());
    double y_error = -(setpoint - limelight.getTy());

    LoggingUtil.logWithNetworkTable(table, "X Err", x_error);
    LoggingUtil.logWithNetworkTable(table, "Y Err", y_error);

    double x_adjust = 0;
    if (Math.abs(x_error) > VisionConstants.ALIGN_THRESHOLD) {
      x_adjust = VisionConstants.ALIGN_KP_X * x_error;
    }
    double y_adjust = VisionConstants.ALIGN_KP_Y * y_error;

    LoggingUtil.logWithNetworkTable(table, "X Adjust", x_adjust);
    LoggingUtil.logWithNetworkTable(table, "Y Adjust", y_adjust);


    double leftSpeed = x_adjust + y_adjust;
    double rightSpeed = y_adjust - x_adjust;
    
    LoggingUtil.logWithNetworkTable(table, "Auto State", "Aligning");
    LoggingUtil.logWithNetworkTable(table, "Left Speed", leftSpeed);
    LoggingUtil.logWithNetworkTable(table, "Right Speed", rightSpeed);

    if (Math.abs(leftSpeed) > Constants.VisionConstants.MAX_ALIGN_SPEED){
      leftSpeed = Math.signum(leftSpeed)*Constants.VisionConstants.MAX_ALIGN_SPEED;
    }
    if (Math.abs(rightSpeed) > Constants.VisionConstants.MAX_ALIGN_SPEED){
      rightSpeed = Math.signum(rightSpeed)*Constants.VisionConstants.MAX_ALIGN_SPEED;
    }
    if (Math.abs(x_error) < 1 && Math.abs(y_error) < 1) {
      if (Robot.isReal()) {
    //     limelight.setLEDMode(1);
        //LoggingUtil.logWithNetworkTable(table, "LED Status", "FORCE OFF");
      }
    } else {
      RobotContainer.m_driveSubsystem.tankDriveAuto(leftSpeed, rightSpeed);
    }
    if (x_adjust < .1 || y_adjust < .1) {
     /* SmartDashboard.putString("AUTO STATE","FINISHED");
      LoggingUtil.logWithNetworkTable(table, "Aligning Status", "FINISHED ALIGNING");
      if (Robot.isReal()) {
        // limelight.setLEDMode(1);
        LoggingUtil.logWithNetworkTable(table, "LED Status", "FORCE OFF");
      }*/
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
    log();
    // update();
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

